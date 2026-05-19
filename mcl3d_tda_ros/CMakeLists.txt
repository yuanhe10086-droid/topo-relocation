cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(mcl3d_tda_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
)

set(ROS2_DEPS
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(mcl src/mcl_node.cpp src/mcl.cpp src/distance_field.cpp src/imu.cpp src/descriptor_db.cpp)
ament_target_dependencies(mcl ${ROS2_DEPS})
target_link_libraries(mcl ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(pc_to_df src/pc_to_df.cpp src/distance_field.cpp)
ament_target_dependencies(pc_to_df rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pc_to_df ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES} OpenMP::OpenMP_CXX)

add_executable(sensor_points_merger src/sensor_points_merger.cpp)
ament_target_dependencies(sensor_points_merger rclcpp sensor_msgs geometry_msgs pcl_conversions tf2 tf2_ros)
target_link_libraries(sensor_points_merger ${PCL_LIBRARIES})

add_executable(build_descriptor_db src/build_descriptor_db.cpp src/descriptor_db.cpp)
target_link_libraries(build_descriptor_db ${PCL_LIBRARIES})

install(TARGETS
  mcl
  pc_to_df
  sensor_points_merger
  build_descriptor_db
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  rviz
  data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_export_include_directories(include)
ament_export_dependencies(${ROS2_DEPS})
ament_package()
