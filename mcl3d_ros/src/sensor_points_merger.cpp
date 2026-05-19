/****************************************************************************
 * mcl3d_ros: 3D Monte Carlo localization for ROS use
 * Copyright (C) 2023 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 ****************************************************************************/

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mcl3d_ros/Pose.h>

class SensorPointsMerger : public rclcpp::Node {
private:
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;

    template<typename T>
    T param(const std::string &name, const T &default_value) {
        return this->declare_parameter<T>(name, default_value);
    }

    std::string baseFrame_;
    std::vector<std::string> sensorFrames_, sensorTopicNames_;
    std::vector<rclcpp::Subscription<PointCloud2Msg>::SharedPtr> pointsSubs_;
    std::vector<mcl3d::Pose> displacements_;
    std::vector<bool> gotPoints_;

    std::string mergedPointsName_, mergedPointsFrame_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr pointsPub_;
    sensor_msgs::msg::PointCloud mergedPoints_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

public:
    SensorPointsMerger(void):
        Node("sensor_points_merger"),
        baseFrame_("base_link"),
        sensorFrames_({"hokuyo3d_front", "hokuyo3d_rear"}),
        sensorTopicNames_({"/cloud", "/cloud"}),
        mergedPointsName_("/velodyne_points"),
        mergedPointsFrame_("velodyne"),
        tfBuffer_(this->get_clock()),
        tfListener_(tfBuffer_)
    {
        baseFrame_ = param<std::string>("base_frame", baseFrame_);
        sensorFrames_ = param<std::vector<std::string>>("sensor_frames", sensorFrames_);
        sensorTopicNames_ = param<std::vector<std::string>>("sensor_topic_names", sensorTopicNames_);
        mergedPointsName_ = param<std::string>("merged_points_name", mergedPointsName_);
        mergedPointsFrame_ = param<std::string>("merged_points_frame", mergedPointsFrame_);

        if (sensorFrames_.size() != sensorTopicNames_.size()) {
            RCLCPP_ERROR(this->get_logger(), "sensor_frames and sensor_topic_names must have the same length.");
            throw std::runtime_error("sensor_frames and sensor_topic_names size mismatch");
        }
        gotPoints_.assign(sensorFrames_.size(), false);

        for (size_t i = 0; i < sensorFrames_.size(); ++i) {
            geometry_msgs::msg::TransformStamped trans;
            int tfFailedCnt = 0;
            rclcpp::Rate loopRate(10.0);
            while (rclcpp::ok()) {
                try {
                    trans = tfBuffer_.lookupTransform(
                        baseFrame_, sensorFrames_[i], tf2::TimePointZero,
                        tf2::durationFromSec(1.0));
                    break;
                } catch (const tf2::TransformException &ex) {
                    (void)ex;
                    tfFailedCnt++;
                    if (tfFailedCnt >= 100) {
                        RCLCPP_ERROR(this->get_logger(),
                            "Cannot get the relative pose from %s to %s. Did you set the static transform publisher?",
                            baseFrame_.c_str(), sensorFrames_[i].c_str());
                        throw std::runtime_error("cannot get sensor transform");
                    }
                    loopRate.sleep();
                }
            }

            tf2::Quaternion quat(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            displacements_.emplace_back(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                roll, pitch, yaw);
        }

        for (size_t i = 0; i < sensorTopicNames_.size(); ++i) {
            pointsSubs_.push_back(this->create_subscription<PointCloud2Msg>(
                sensorTopicNames_[i], rclcpp::SensorDataQoS(),
                [this, i](PointCloud2Msg::SharedPtr msg) { this->pointsCB(msg, i); }));
        }

        pointsPub_ = this->create_publisher<PointCloud2Msg>(mergedPointsName_, 5);
        mergedPoints_.points.clear();
        RCLCPP_INFO(this->get_logger(), "Initialization has been done.");
    }

    void pointsCB(const PointCloud2Msg::SharedPtr msg, size_t sensorIndex) {
        if (sensorIndex >= sensorFrames_.size()) {
            return;
        }
        if (msg->header.frame_id != sensorFrames_[sensorIndex]) {
            return;
        }
        if (gotPoints_[sensorIndex]) {
            return;
        }

        mcl3d::Pose &poseTrans = displacements_[sensorIndex];
        gotPoints_[sensorIndex] = true;

        double cr = cos(poseTrans.getRoll());
        double sr = sin(poseTrans.getRoll());
        double cp = cos(poseTrans.getPitch());
        double sp = sin(poseTrans.getPitch());
        double cy = cos(poseTrans.getYaw());
        double sy = sin(poseTrans.getYaw());

        double m11 = cy * cp;
        double m12 = cy * sp * sr - sy * cr;
        double m13 = sy * sr + cy * sp * cr;
        double m21 = sy * cp;
        double m22 = cy * cr + sy * sp * sr;
        double m23 = sy * sp * cr - cy * sr;
        double m31 = -sp;
        double m32 = cp * sr;
        double m33 = cp * cr;

        pcl::PointCloud<pcl::PointXYZ> points;
        pcl::fromROSMsg(*msg, points);

        for (size_t i = 0; i < points.size(); ++i) {
            float x = points.points[i].x;
            float y = points.points[i].y;
            float z = points.points[i].z;
            geometry_msgs::msg::Point32 p;
            p.x = x * m11 + y * m12 + z * m13 + poseTrans.getX();
            p.y = x * m21 + y * m22 + z * m23 + poseTrans.getY();
            p.z = x * m31 + y * m32 + z * m33 + poseTrans.getZ();
            mergedPoints_.points.push_back(p);
        }

        for (bool gotPoints : gotPoints_) {
            if (!gotPoints) {
                return;
            }
        }

        pcl::PointCloud<pcl::PointXYZ> mergedPcl;
        mergedPcl.points.reserve(mergedPoints_.points.size());
        for (const auto &point : mergedPoints_.points) {
            mergedPcl.points.emplace_back(point.x, point.y, point.z);
        }
        mergedPcl.width = static_cast<uint32_t>(mergedPcl.points.size());
        mergedPcl.height = 1;
        mergedPcl.is_dense = false;

        PointCloud2Msg sensorPoints;
        pcl::toROSMsg(mergedPcl, sensorPoints);
        sensorPoints.header.stamp = this->now();
        sensorPoints.header.frame_id = mergedPointsFrame_;
        pointsPub_->publish(sensorPoints);

        mergedPoints_.points.clear();
        std::fill(gotPoints_.begin(), gotPoints_.end(), false);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SensorPointsMerger>());
    } catch (const std::exception &e) {
        fprintf(stderr, "sensor_points_merger initialization failed: %s\n", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
