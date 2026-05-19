# mcl3d_tda_ros

Experimental ROS 2 package copied from `mcl3d_ros` for descriptor-assisted MCL3D.

Current prototype:

- offline descriptor DB generation from a global PCD map
- lightweight gravity-aligned height-layer/range descriptor
- top-K descriptor retrieval for uninitialized particle initialization
- descriptor likelihood update for initialized particles
- original MCL3D distance-field localization remains available

The TDA descriptor is not implemented yet. The current descriptor is a lightweight placeholder so the whole pipeline can be tested first.

## Build

```bash
cd /home/walli/topo_relocation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mcl3d_tda_ros
source install/setup.bash
```

## Offline

Build the distance-field map:

```bash
ros2 run mcl3d_tda_ros pc_to_df \
  /home/walli/topo_relocation_ws/src/mcl3d_tda_ros/data/map_mcl_3dl.pcd \
  tda_map.bin \
  5.0 \
  0.1 \
  1.0 \
  /tmp/tda_map.yaml
```

Build the descriptor DB:

```bash
ros2 run mcl3d_tda_ros build_descriptor_db \
  /home/walli/topo_relocation_ws/src/mcl3d_tda_ros/data/map_mcl_3dl.pcd \
  /tmp/mcl3d_tda_desc.csv \
  2.0 \
  45.0 \
  10.0 \
  6 \
  8 \
  -1.5 \
  3.0
```

Arguments are:

```text
pcd_file descriptor_csv xy_resolution yaw_resolution_deg radius height_layers range_bins min_z max_z
```

## Online

Run MCL3D with descriptor-assisted initialization and likelihood update:

```bash
ros2 launch mcl3d_tda_ros mcl.launch.py \
  map_yaml_file:=/tmp/tda_map.yaml \
  use_descriptor_localization:=true \
  descriptor_db_file:=/tmp/mcl3d_tda_desc.csv \
  sensor_points_name:=/velodyne_points
```

Important descriptor parameters:

```text
descriptor_top_k
descriptor_radius
descriptor_height_layers
descriptor_range_bins
descriptor_min_z
descriptor_max_z
descriptor_sigma
descriptor_pose_xy_weight
descriptor_pose_yaw_weight
```

## Next Steps

Replace `DescriptorDB::computeHeightLayerDescriptor()` with the real TDA + geometry descriptor while keeping the same DB/query/MCL interfaces.

# 中文说明

`mcl3d_tda_ros` 是从 `mcl3d_ros` 复制出来的 ROS 2 实验包，用来测试“描述子辅助的 MCL3D”方案。

当前原型已经包含：

- 从全局 PCD 地图离线生成描述子库
- 轻量级 height-layer/range 描述子
- 未初始化时通过描述子库 top-K 检索初始化粒子
- 已初始化时通过描述子似然更新粒子权重
- 保留原始 MCL3D 的 distance field 几何定位能力

注意：真正的 TDA 描述子还没有实现。当前描述子只是一个轻量占位版本，目的是先把完整流程跑通，后续再替换成 `TDA + 几何复合描述子`。

## 编译

```bash
cd /home/walli/topo_relocation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mcl3d_tda_ros
source install/setup.bash
```

## 离线阶段

先生成 distance-field 地图：

```bash
ros2 run mcl3d_tda_ros pc_to_df \
  /home/walli/topo_relocation_ws/src/mcl3d_tda_ros/data/map_mcl_3dl.pcd \
  tda_map.bin \
  5.0 \
  0.1 \
  1.0 \
  /tmp/tda_map.yaml
```

再生成描述子库：

```bash
ros2 run mcl3d_tda_ros build_descriptor_db \
  /home/walli/topo_relocation_ws/src/mcl3d_tda_ros/data/map_mcl_3dl.pcd \
  /tmp/mcl3d_tda_desc.csv \
  2.0 \
  45.0 \
  10.0 \
  6 \
  8 \
  -1.5 \
  3.0
```

参数含义：

```text
pcd_file descriptor_csv xy_resolution yaw_resolution_deg radius height_layers range_bins min_z max_z
```

对应解释：

```text
pcd_file             全局 PCD 地图
descriptor_csv       输出的描述子库 CSV
xy_resolution        地图位姿 XY 采样间隔，单位 m
yaw_resolution_deg   yaw 采样间隔，单位 deg
radius               局部子地图半径，单位 m
height_layers        高度分层数量
range_bins           距离分桶数量
min_z                描述子使用的最低相对高度
max_z                描述子使用的最高相对高度
```

## 在线阶段

启动带描述子辅助的 MCL3D：

```bash
ros2 launch mcl3d_tda_ros mcl.launch.py \
  map_yaml_file:=/tmp/tda_map.yaml \
  use_descriptor_localization:=true \
  descriptor_db_file:=/tmp/mcl3d_tda_desc.csv \
  sensor_points_name:=/velodyne_points
```

如果你的 LiDAR 点云话题不是 `/velodyne_points`，需要改成自己的话题：

```bash
sensor_points_name:=/你的点云话题
```

重要描述子参数：

```text
descriptor_top_k              top-K 检索候选数量
descriptor_radius             在线描述子局部半径
descriptor_height_layers      高度分层数量
descriptor_range_bins         距离分桶数量
descriptor_min_z              最低相对高度
descriptor_max_z              最高相对高度
descriptor_sigma              描述子似然的 sigma
descriptor_pose_xy_weight     查询粒子附近地图描述子时的 XY 权重
descriptor_pose_yaw_weight    查询粒子附近地图描述子时的 yaw 权重
```

## 当前流程

离线：

```text
全局 3D 点云地图
-> 可行走区域/地图位姿采样
-> 局部 3D 子地图
-> height-layer descriptor
-> 描述子库
-> distance field map
```

在线：

```text
LiDAR 点云输入
-> 范围裁剪/局部描述子
-> 描述子库 top-K 检索
-> 未初始化时用 top-K 初始化粒子
-> 已初始化时用描述子似然更新粒子权重
-> MCL3D distance field 几何更新
-> 输出 T_map_base_link
```

## 后续工作

下一步应替换：

```text
DescriptorDB::computeHeightLayerDescriptor()
```

把当前轻量占位描述子替换成真正的：

```text
TDA + 几何复合描述子
```

同时保留现有的描述子库读取、top-K 查询和 MCL 粒子更新接口。
