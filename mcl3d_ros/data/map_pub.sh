#! /bin/bash

pcd_file=$1
publish_cycle=10.0
frame_id="map"
topic_name="/map_points"

ros2 run pcl_ros pcd_to_pointcloud "$pcd_file" "$publish_cycle" --ros-args \
  -p frame_id:="$frame_id" \
  -r cloud_pcd:="$topic_name"
