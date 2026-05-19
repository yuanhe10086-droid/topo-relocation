/****************************************************************************
 * mcl3d_ros: 3D Monte Carlo localization for ROS use
 * Copyright (C) 2023 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 ****************************************************************************/

#include <cstdlib>
#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mcl3d_tda_ros/DistanceField.h>

mcl3d::DistanceField distMap;
bool doneMapBuild = false;

std::string mapFileName, yamlFilePath;
float resolution, subMapResolution, mapMargin;

bool buildDistanceField(pcl::PointCloud<pcl::PointXYZ> mapPoints, std::string mapFileName,
    float resolution, float subMapResolution, float mapMargin, std::string yamlFilePath)
{
    if (mapPoints.empty()) {
        fprintf(stderr, "The input map point cloud is empty.\n");
        return false;
    }

    mcl3d::Point minPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    mcl3d::Point maxPoint(mapPoints.points[0].x, mapPoints.points[0].y, mapPoints.points[0].z);
    for (int i = 1; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints[i].x;
        float y = mapPoints[i].y;
        float z = mapPoints[i].z;
        if (minPoint.getX() > x)
            minPoint.setX(x);
        if (minPoint.getY() > y)
            minPoint.setY(y);
        if (minPoint.getZ() > z)
            minPoint.setZ(z);
        if (maxPoint.getX() < x)
            maxPoint.setX(x);
        if (maxPoint.getY() < y)
            maxPoint.setY(y);
        if (maxPoint.getZ() < z)
            maxPoint.setZ(z);
    }
    printf("Min map point: %f %f %f\n", minPoint.getX(), minPoint.getY(), minPoint.getZ());
    printf("Max map point: %f %f %f\n", maxPoint.getX(), maxPoint.getY(), maxPoint.getZ());

    distMap = mcl3d::DistanceField(mapFileName, resolution, subMapResolution, mapMargin, minPoint, maxPoint, yamlFilePath);
    for (int i = 0; i < (int)mapPoints.size(); ++i) {
        float x = mapPoints.points[i].x;
        float y = mapPoints.points[i].y;
        float z = mapPoints.points[i].z;
        distMap.addPoint(x, y, z);
    }

    if (!distMap.saveDistanceMap()) {
        fprintf(stderr, "Error occurred during the distance field building.\n");
        return false;
    }

    return true;
}

class PcToDfNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapPointsSub_;

public:
    PcToDfNode(void): Node("pc_to_df") {
        std::string mapPointsName = this->declare_parameter<std::string>("map_points_name", "/map_points");

        mapFileName = this->declare_parameter<std::string>("map_file_name", "dist_map.bin");
        resolution = static_cast<float>(this->declare_parameter<double>("resolution", 5.0));
        subMapResolution = static_cast<float>(this->declare_parameter<double>("sub_map_resolution", 0.1));
        mapMargin = static_cast<float>(this->declare_parameter<double>("map_margin", 1.0));
        yamlFilePath = this->declare_parameter<std::string>("yaml_file_path", "/tmp/dist_map.yaml");

        printf("mapPointsName: %s\n", mapPointsName.c_str());
        printf("mapFileName: %s\n", mapFileName.c_str());
        printf("resolution: %f [m]\n", resolution);
        printf("subMapResolution: %f [m]\n", subMapResolution);
        printf("mapMargin: %f [m]\n", mapMargin);
        printf("yamlFilePath: %s\n", yamlFilePath.c_str());

        mapPointsSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            mapPointsName, rclcpp::SensorDataQoS(),
            std::bind(&PcToDfNode::mapPointsCB, this, std::placeholders::_1));
    }

    void mapPointsCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> mapPoints;
        pcl::fromROSMsg(*msg, mapPoints);
        doneMapBuild = buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath);
        if (doneMapBuild) {
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char **argv) {
    if (argc >= 7) {
        std::string pcdFile = argv[1];
        mapFileName = argv[2];
        resolution = std::atof(argv[3]);
        subMapResolution = std::atof(argv[4]);
        mapMargin = std::atof(argv[5]);
        yamlFilePath = argv[6];

        printf("pcdFile: %s\n", pcdFile.c_str());
        printf("mapFileName: %s\n", mapFileName.c_str());
        printf("resolution: %f [m]\n", resolution);
        printf("subMapResolution: %f [m]\n", subMapResolution);
        printf("mapMargin: %f [m]\n", mapMargin);
        printf("yamlFilePath: %s\n", yamlFilePath.c_str());

        pcl::PointCloud<pcl::PointXYZ> mapPoints;
        if (pcl::io::loadPCDFile(pcdFile, mapPoints) != 0) {
            fprintf(stderr, "Cannot read PCD file -> %s\n", pcdFile.c_str());
            return 1;
        }
        return buildDistanceField(mapPoints, mapFileName, resolution, subMapResolution, mapMargin, yamlFilePath) ? 0 : 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcToDfNode>());
    rclcpp::shutdown();
    return 0;
}
