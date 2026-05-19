/****************************************************************************
 * mcl3d_ros: 3D Monte Carlo localization for ROS use
 * Copyright (C) 2023 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 ****************************************************************************/

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mcl3d_ros/IMU.h>
#include <mcl3d_ros/MCL.h>

class MCLNode : public rclcpp::Node {
private:
    template<typename T>
    T param(const std::string &name, const T &default_value) {
        return this->declare_parameter<T>(name, default_value);
    }

    using PointCloudMsg = sensor_msgs::msg::PointCloud;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using ImuMsg = sensor_msgs::msg::Imu;
    using OdometryMsg = nav_msgs::msg::Odometry;
    using PoseWithCovarianceStampedMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
    using PoseArrayMsg = geometry_msgs::msg::PoseArray;

    rclcpp::Subscription<PointCloud2Msg>::SharedPtr sensorPointsSub_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imuSub_;
    rclcpp::Subscription<OdometryMsg>::SharedPtr odomSub_;
    rclcpp::Subscription<PoseWithCovarianceStampedMsg>::SharedPtr initialPoseSub_;

    rclcpp::Publisher<PoseArrayMsg>::SharedPtr particlesPub_;
    rclcpp::Publisher<PoseArrayMsg>::SharedPtr optParticlesPub_;
    rclcpp::Publisher<PoseWithCovarianceStampedMsg>::SharedPtr posePub_;
    rclcpp::Publisher<PoseWithCovarianceStampedMsg>::SharedPtr optPosePub_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr alignedPointsOptPub_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr mapPointsPub_;

    std::string mapFrame_, odomFrame_, baseLinkFrame_, laserFrame_, optPoseFrame_;
    mcl3d::MCL mcl_;
    mcl3d::Pose initialPose_, initialNoise_;
    mcl3d::IMU imu_;
    rclcpp::Time mclPoseStamp_;
    double transformTolerance_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    bool broadcastTF_, useOdomTF_;
    std::string logFile_;
    bool writeLog_;

public:
    MCLNode(void):
        Node("mcl"),
        mapFrame_("map"),
        odomFrame_("odom"),
        baseLinkFrame_("base_link"),
        laserFrame_("laser"),
        optPoseFrame_("opt_pose"),
        mclPoseStamp_(this->now()),
        transformTolerance_(0.0),
        tfBroadcaster_(this),
        broadcastTF_(true),
        useOdomTF_(false),
        logFile_("/tmp/mcl3d_log.txt"),
        writeLog_(false)
    {
        std::string sensorPointsName = param<std::string>("sensor_points_name", "/velodyne_points");
        std::string imuName = param<std::string>("imu_name", "/imu/data");
        std::string odomName = param<std::string>("odom_name", "/odom");
        bool useIMU = param<bool>("use_imu", false);
        bool useOdom = param<bool>("use_odom", false);
        bool useInitialPoseCB = param<bool>("use_initial_pose_cb", true);
        useOdomTF_ = param<bool>("use_odom_tf", useOdomTF_);

        sensorPointsSub_ = this->create_subscription<PointCloud2Msg>(
            sensorPointsName, rclcpp::SensorDataQoS(),
            std::bind(&MCLNode::sensorPointsCB, this, std::placeholders::_1));
        if (useIMU) {
            imuSub_ = this->create_subscription<ImuMsg>(
                imuName, rclcpp::SensorDataQoS(),
                std::bind(&MCLNode::imuCB, this, std::placeholders::_1));
        }
        if (useOdom || useOdomTF_) {
            odomSub_ = this->create_subscription<OdometryMsg>(
                odomName, 10, std::bind(&MCLNode::odomCB, this, std::placeholders::_1));
        }
        if (useInitialPoseCB) {
            initialPoseSub_ = this->create_subscription<PoseWithCovarianceStampedMsg>(
                "/initialpose", 1, std::bind(&MCLNode::initialPoseCB, this, std::placeholders::_1));
        }

        std::string particlesName = param<std::string>("particles_name", "/particles");
        std::string optParticlesName = param<std::string>("opt_particles_name", "/optimized_particles");
        std::string poseName = param<std::string>("pose_name", "/mcl_pose");
        std::string optPoseName = param<std::string>("opt_pose_name", "/opt_pose");
        std::string alignedPointsOptName = param<std::string>("aligned_points_opt_name", "/aligned_points_opt");
        std::string mapPointsName = param<std::string>("map_points_name", "/df_map_points");

        particlesPub_ = this->create_publisher<PoseArrayMsg>(particlesName, 1);
        optParticlesPub_ = this->create_publisher<PoseArrayMsg>(optParticlesName, 1);
        posePub_ = this->create_publisher<PoseWithCovarianceStampedMsg>(poseName, 1);
        optPosePub_ = this->create_publisher<PoseWithCovarianceStampedMsg>(optPoseName, 1);
        alignedPointsOptPub_ = this->create_publisher<PointCloudMsg>(alignedPointsOptName, 1);
        mapPointsPub_ = this->create_publisher<PointCloudMsg>(
            mapPointsName, rclcpp::QoS(1).transient_local().reliable());

        mapFrame_ = param<std::string>("map_frame", mapFrame_);
        odomFrame_ = param<std::string>("odom_frame", odomFrame_);
        baseLinkFrame_ = param<std::string>("base_link_frame", baseLinkFrame_);
        laserFrame_ = param<std::string>("laser_frame", laserFrame_);
        optPoseFrame_ = param<std::string>("opt_pose_frame", optPoseFrame_);

        int localizationMode = param<int>("localization_mode", 0);
        int measurementModelType = param<int>("measurement_model_type", 3);
        mcl_.setLocalizationMode(localizationMode);
        mcl_.setMeasurementModelType(measurementModelType);

        mcl_.setParticleNum(param<int>("particle_num", 500));
        mcl_.setSensorPointsNum(param<int>("sensor_points_num", 200));
        mcl_.setVoxelLeafSize(param<double>("voxel_leaf_size", 1.0));

        std::vector<double> initialPose = param<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        std::vector<double> initialNoise = param<std::vector<double>>("initial_noise", {0.1, 0.1, 0.1, 0.05, 0.05, 0.05});
        std::vector<double> baseLink2Laser = param<std::vector<double>>("base_link_2_laser", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        if (initialPose.size() != 6 || initialNoise.size() != 6 || baseLink2Laser.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "initial_pose, initial_noise, and base_link_2_laser must each contain 6 values.");
            throw std::runtime_error("invalid pose parameter size");
        }

        initialPose[3] *= M_PI / 180.0;
        initialPose[4] *= M_PI / 180.0;
        initialPose[5] *= M_PI / 180.0;
        baseLink2Laser[3] *= M_PI / 180.0;
        baseLink2Laser[4] *= M_PI / 180.0;
        baseLink2Laser[5] *= M_PI / 180.0;
        initialPose_.setPose(initialPose[0], initialPose[1], initialPose[2], initialPose[3], initialPose[4], initialPose[5]);
        initialNoise_.setPose(initialNoise[0], initialNoise[1], initialNoise[2], initialNoise[3], initialNoise[4], initialNoise[5]);
        mcl3d::Pose baseLink2LaserPose(baseLink2Laser[0], baseLink2Laser[1], baseLink2Laser[2],
            baseLink2Laser[3], baseLink2Laser[4], baseLink2Laser[5]);
        mcl_.setInitialPose(initialPose_);
        mcl_.setBaseLink2Laser(baseLink2LaserPose);
        mcl_.initializeParticles(initialPose_, initialNoise_);

        if (useIMU) {
            imu_.init();
            imu_.setSampleFreq(param<double>("imu_sample_freq", 100.0));
            imu_.setAHRSFilterGains(param<double>("ahrs_filter_kp", 0.0), param<double>("ahrs_filter_ki", 0.0));
        } else {
            (void)param<double>("imu_sample_freq", 100.0);
            (void)param<double>("ahrs_filter_kp", 0.0);
            (void)param<double>("ahrs_filter_ki", 0.0);
        }

        mcl_.setMeasurementModelParameters(
            param<double>("z_hit", 0.9),
            param<double>("z_rand", 0.05),
            param<double>("z_max", 0.05),
            param<double>("var_hit", 0.01),
            param<double>("unknown_lambda", 0.001),
            param<double>("range_reso", 0.1),
            param<double>("range_max", 120.0));

        std::vector<double> odomNoise = param<std::vector<double>>("odom_noise", {
            1.0, 0.1, 0.1, 0.1, 0.1, 0.1,
            0.1, 1.0, 0.1, 0.1, 0.1, 0.1,
            0.1, 0.1, 1.0, 0.1, 0.1, 0.1,
            0.1, 0.1, 0.1, 1.0, 0.1, 0.1,
            0.1, 0.1, 0.1, 0.1, 1.0, 0.1,
            0.1, 0.1, 0.1, 0.1, 0.1, 1.0});
        mcl_.setOdomNoise(odomNoise);

        mcl_.setUseLinearInterpolation(param<bool>("use_linear_interpolation", true));
        double randomParticleRate = param<double>("random_particle_rate", 0.1);
        double resampleThreshold = param<double>("resample_threshold", 0.5);
        std::vector<double> resampleNoise = param<std::vector<double>>("resample_noise", {0.1, 0.1, 0.1, 0.05, 0.05, 0.05});
        mcl3d::Pose resampleNoisePose(resampleNoise[0], resampleNoise[1], resampleNoise[2],
            resampleNoise[3], resampleNoise[4], resampleNoise[5]);
        mcl_.setRandomParticleRate(randomParticleRate);
        mcl_.setResampleThreshold(resampleThreshold);
        mcl_.setResampleNoise(resampleNoisePose);

        mcl_.setOptMaxIterNum(param<int>("opt_max_iter_num", 30));
        mcl_.setOptMaxError(param<double>("opt_max_error", 1.0));
        mcl_.setConvergenceThreshold(param<double>("convergence_threshold", 0.02));
        mcl_.setOptParticlsNum(param<int>("optimized_particle_num", 100));
        mcl_.setOptPoseCovCoef(param<double>("optimized_pose_cov_coef", 1.0));
        mcl_.setGMMPosVar(param<double>("gmm_postion_var", 0.3));
        mcl_.setGMMAngVar(param<double>("gmm_angle_var", 0.1));

        transformTolerance_ = param<double>("transform_tolerance", transformTolerance_);
        broadcastTF_ = param<bool>("broadcast_tf", broadcastTF_);

        if (!mcl_.checkParameters()) {
            RCLCPP_ERROR(this->get_logger(), "Incorrect parameters are given for MCL.");
            throw std::runtime_error("invalid MCL parameters");
        }

        std::string mapYamlFile = param<std::string>("map_yaml_file", "/tmp/dist_map.yaml");
        RCLCPP_INFO(this->get_logger(), "The given map yaml file is %s. Start map loading.", mapYamlFile.c_str());
        if (!mcl_.loadDistanceMap(mapYamlFile)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot read map yaml file -> %s", mapYamlFile.c_str());
            throw std::runtime_error("cannot read map yaml file");
        }

        PointCloudMsg mapPointsMsg;
        std::vector<mcl3d::Point> mapPoints = mcl_.getMapPoints();
        mapPointsMsg.header.frame_id = mapFrame_;
        mapPointsMsg.header.stamp = this->now();
        mapPointsMsg.points.resize(mapPoints.size());
        for (size_t i = 0; i < mapPoints.size(); ++i) {
            geometry_msgs::msg::Point32 p;
            p.x = mapPoints[i].getX();
            p.y = mapPoints[i].getY();
            p.z = mapPoints[i].getZ();
            mapPointsMsg.points[i] = p;
        }
        mapPointsPub_->publish(mapPointsMsg);

        logFile_ = param<std::string>("log_file", logFile_);
        writeLog_ = param<bool>("write_log", writeLog_);
        RCLCPP_INFO(this->get_logger(), "Initialization for MCL has been done.");
    }

    void sensorPointsCB(const PointCloud2Msg::SharedPtr msg) {
        static double totalTime = 0.0;
        static int count = 0;

        mclPoseStamp_ = msg->header.stamp;
        pcl::PointCloud<pcl::PointXYZ> sensorPointsTmp;
        pcl::fromROSMsg(*msg, sensorPointsTmp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPoints(new pcl::PointCloud<pcl::PointXYZ>);
        *sensorPoints = sensorPointsTmp;

        mcl_.updatePoses();
        auto start = std::chrono::system_clock::now();
        mcl_.calculateLikelihoodsByMeasurementModel(sensorPoints);
        mcl_.optimizeMeasurementModel(sensorPoints);
        mcl_.resampleParticles1();
        mcl_.resampleParticles2();
        auto end = std::chrono::system_clock::now();
        double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
        totalTime += time;
        count++;
        printf("time = %lf, average = %lf\n", time, totalTime / static_cast<double>(count));
        mcl_.printMCLResult();
        publishROSMessages();
        broadcastTF();
        writeLog();
    }

    void imuCB(const ImuMsg::SharedPtr msg) {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (norm > 0.99) {
            tf2::Quaternion q(qx, qy, qz, qw);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            imu_.setRPY(roll, pitch, yaw);
        } else {
            imu_.setAcceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu_.setAngularVelocities(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            imu_.updateOrientation();
        }
        mcl_.setIMU(imu_);
    }

    void odomCB(const OdometryMsg::SharedPtr msg) {
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        mcl3d::Pose odomPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z, roll, pitch, yaw);
        mcl3d::Pose odomVel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
            msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        mcl_.setOdomPose(odomPose);
        mcl_.setOdomVelocities(odomVel);
    }

    void initialPoseCB(const PoseWithCovarianceStampedMsg::SharedPtr msg) {
        tf2::Quaternion q(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        mcl3d::Pose initialPose(msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z, roll, pitch, yaw);
        mcl_.setInitialPose(initialPose);
        mcl_.initializeParticles(initialPose, initialNoise_);
    }

    void publishROSMessages(void) {
        std::vector<mcl3d::Particle> particles = mcl_.getParticles();
        PoseArrayMsg particlesPoses;
        particlesPoses.header.frame_id = mapFrame_;
        particlesPoses.header.stamp = mclPoseStamp_;
        particlesPoses.poses.resize(particles.size());
        for (size_t i = 0; i < particles.size(); ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = particles[i].getX();
            pose.position.y = particles[i].getY();
            pose.position.z = particles[i].getZ();
            tf2::Quaternion q;
            q.setRPY(particles[i].getRoll(), particles[i].getPitch(), particles[i].getYaw());
            pose.orientation = tf2::toMsg(q);
            particlesPoses.poses[i] = pose;
        }
        particlesPub_->publish(particlesPoses);

        std::vector<mcl3d::Particle> optParticles = mcl_.getOptParticles();
        PoseArrayMsg optParticlesPoses;
        optParticlesPoses.header.frame_id = mapFrame_;
        optParticlesPoses.header.stamp = mclPoseStamp_;
        optParticlesPoses.poses.resize(optParticles.size());
        for (size_t i = 0; i < optParticles.size(); ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = optParticles[i].getX();
            pose.position.y = optParticles[i].getY();
            pose.position.z = optParticles[i].getZ();
            tf2::Quaternion q;
            q.setRPY(optParticles[i].getRoll(), optParticles[i].getPitch(), optParticles[i].getYaw());
            pose.orientation = tf2::toMsg(q);
            optParticlesPoses.poses[i] = pose;
        }
        optParticlesPub_->publish(optParticlesPoses);

        PoseWithCovarianceStampedMsg mclPoseMsg;
        PoseWithCovarianceStampedMsg optPoseMsg;
        mcl3d::Pose mclPose = mcl_.getMCLPose();
        mcl3d::Pose optPose = mcl_.getOptPose();

        mclPoseMsg.header.frame_id = mapFrame_;
        mclPoseMsg.header.stamp = mclPoseStamp_;
        mclPoseMsg.pose.pose.position.x = mclPose.getX();
        mclPoseMsg.pose.pose.position.y = mclPose.getY();
        mclPoseMsg.pose.pose.position.z = mclPose.getZ();
        tf2::Quaternion mclQuat;
        mclQuat.setRPY(mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw());
        mclPoseMsg.pose.pose.orientation = tf2::toMsg(mclQuat);

        optPoseMsg.header.frame_id = mapFrame_;
        optPoseMsg.header.stamp = mclPoseStamp_;
        optPoseMsg.pose.pose.position.x = optPose.getX();
        optPoseMsg.pose.pose.position.y = optPose.getY();
        optPoseMsg.pose.pose.position.z = optPose.getZ();
        tf2::Quaternion optQuat;
        optQuat.setRPY(optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
        optPoseMsg.pose.pose.orientation = tf2::toMsg(optQuat);

        std::vector<std::vector<double>> poseCov = mcl_.getPoseCovariance();
        std::vector<std::vector<double>> optPoseCov = mcl_.getOptPoseCovariance();
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                mclPoseMsg.pose.covariance[j * 6 + i] = poseCov[i][j];
                optPoseMsg.pose.covariance[j * 6 + i] = optPoseCov[i][j];
            }
        }
        posePub_->publish(mclPoseMsg);
        optPosePub_->publish(optPoseMsg);

        pcl::PointCloud<pcl::PointXYZ> alignedPointsOpt = mcl_.getAlignedPointsOpt();
        PointCloudMsg alignedPointsOptMsg;
        alignedPointsOptMsg.points.resize(alignedPointsOpt.points.size());
        for (size_t i = 0; i < alignedPointsOpt.points.size(); ++i) {
            geometry_msgs::msg::Point32 p;
            p.x = alignedPointsOpt.points[i].x;
            p.y = alignedPointsOpt.points[i].y;
            p.z = alignedPointsOpt.points[i].z;
            alignedPointsOptMsg.points[i] = p;
        }
        alignedPointsOptMsg.header.frame_id = laserFrame_;
        alignedPointsOptMsg.header.stamp = mclPoseStamp_;
        alignedPointsOptPub_->publish(alignedPointsOptMsg);
    }

    void broadcastTF(void) {
        if (!broadcastTF_) {
            return;
        }

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        geometry_msgs::msg::Pose poseOnMap;
        poseOnMap.position.x = mclPose.getX();
        poseOnMap.position.y = mclPose.getY();
        poseOnMap.position.z = mclPose.getZ();
        tf2::Quaternion mclQuat;
        mclQuat.setRPY(mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw());
        poseOnMap.orientation = tf2::toMsg(mclQuat);
        tf2::Transform map2baseTrans;
        tf2::convert(poseOnMap, map2baseTrans);

        if (useOdomTF_) {
            mcl3d::Pose odomPose = mcl_.getOdomPose();
            geometry_msgs::msg::Pose poseOnOdom;
            poseOnOdom.position.x = odomPose.getX();
            poseOnOdom.position.y = odomPose.getY();
            poseOnOdom.position.z = odomPose.getZ();
            tf2::Quaternion odomQuat;
            odomQuat.setRPY(odomPose.getRoll(), odomPose.getPitch(), odomPose.getYaw());
            poseOnOdom.orientation = tf2::toMsg(odomQuat);
            tf2::Transform odom2baseTrans;
            tf2::convert(poseOnOdom, odom2baseTrans);

            tf2::Transform map2odomTrans = map2baseTrans * odom2baseTrans.inverse();
            geometry_msgs::msg::TransformStamped map2odomStampedTrans;
            map2odomStampedTrans.header.stamp = mclPoseStamp_ + rclcpp::Duration::from_seconds(transformTolerance_);
            map2odomStampedTrans.header.frame_id = mapFrame_;
            map2odomStampedTrans.child_frame_id = odomFrame_;
            tf2::convert(map2odomTrans, map2odomStampedTrans.transform);
            tfBroadcaster_.sendTransform(map2odomStampedTrans);
        } else {
            geometry_msgs::msg::TransformStamped map2baseStampedTrans;
            map2baseStampedTrans.header.stamp = mclPoseStamp_;
            map2baseStampedTrans.header.frame_id = mapFrame_;
            map2baseStampedTrans.child_frame_id = baseLinkFrame_;
            tf2::convert(map2baseTrans, map2baseStampedTrans.transform);
            tfBroadcaster_.sendTransform(map2baseStampedTrans);
        }

        mcl3d::Pose optPose = mcl_.getOptPose();
        geometry_msgs::msg::Pose optPoseOnMap;
        optPoseOnMap.position.x = optPose.getX();
        optPoseOnMap.position.y = optPose.getY();
        optPoseOnMap.position.z = optPose.getZ();
        tf2::Quaternion optQuat;
        optQuat.setRPY(optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
        optPoseOnMap.orientation = tf2::toMsg(optQuat);
        tf2::Transform map2optPoseTrans;
        tf2::convert(optPoseOnMap, map2optPoseTrans);
        geometry_msgs::msg::TransformStamped map2optPoseStampedTrans;
        map2optPoseStampedTrans.header.stamp = mclPoseStamp_;
        map2optPoseStampedTrans.header.frame_id = mapFrame_;
        map2optPoseStampedTrans.child_frame_id = optPoseFrame_;
        tf2::convert(map2optPoseTrans, map2optPoseStampedTrans.transform);
        tfBroadcaster_.sendTransform(map2optPoseStampedTrans);
    }

    void writeLog(void) {
        if (!writeLog_) {
            return;
        }

        static FILE *fp = nullptr;
        if (fp == nullptr) {
            fp = fopen(logFile_.c_str(), "w");
            if (fp == nullptr) {
                fprintf(stderr, "Cannot open %s\n", logFile_.c_str());
                return;
            }
        }

        mcl3d::Pose mclPose = mcl_.getMCLPose();
        mcl3d::Pose optPose = mcl_.getOptPose();
        fprintf(fp, "%lf "
            "%lf %lf %lf %lf %lf %lf "
            "%lf %lf %lf %lf %lf %lf\n",
            mclPoseStamp_.seconds(),
            mclPose.getX(), mclPose.getY(), mclPose.getZ(), mclPose.getRoll(), mclPose.getPitch(), mclPose.getYaw(),
            optPose.getX(), optPose.getY(), optPose.getZ(), optPose.getRoll(), optPose.getPitch(), optPose.getYaw());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MCLNode>());
    } catch (const std::exception &e) {
        fprintf(stderr, "mcl node initialization failed: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
