/****************************************************************************
 * Structure-aware observation prior for mcl3d_ros
 ****************************************************************************/

#ifndef __STRUCTURE_MAP_H__
#define __STRUCTURE_MAP_H__

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mcl3d_ros/Point.h>

namespace mcl3d {

struct StructureMapParameters {
    // BEV 栅格分辨率，用于统计 occupancy、density、height variance 等结构变化。
    double bevResolution = 0.3;
    // 三维 voxel graph 的体素大小，影响拓扑图节点数量。
    double voxelSize = 0.3;
    // 每个结构兴趣区域周围截取局部点云的半径。
    double localRadius = 6.0;
    // BEV 结构变化分数阈值，越大越偏向明显边界、瓶颈、拐角等区域。
    double candidateScoreThreshold = 0.35;
    // 候选结构区域之间的最小距离，用于抑制过密候选点。
    double candidateMinDistance = 2.0;
    // 局部区域最少点数，点太少时结构向量不可靠。
    int minRegionPoints = 30;
};

class StructureMap {
public:
    using StructureVector = std::array<double, 8>;

    struct Region {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double score = 0.0;
        StructureVector descriptor{};
    };

    StructureMap(void);

    void setParameters(const StructureMapParameters &params);
    // 从全局地图点云生成结构先验 D_map(x)。
    bool buildFromMapPoints(const std::vector<Point> &mapPoints);
    bool isAvailable(void) const;
    int getRegionCount(void) const;

    // 在线 LiDAR 局部观测生成 D_local。
    StructureVector extractFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) const;
    // 根据粒子位置附近的 D_map(x_i) 与 D_local 计算结构似然。
    double likelihood(double x, double y, const StructureVector &localDescriptor, double sigma) const;

private:
    struct BevCell {
        int count = 0;
        double zSum = 0.0;
        double z2Sum = 0.0;
        double maxZ = -1.0e9;
        double score = 0.0;
        double cx = 0.0;
        double cy = 0.0;
    };

    struct VoxelKey {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const VoxelKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey &key) const;
    };

    StructureMapParameters params_;
    std::vector<Point> mapPoints_;
    // candidate_structural_regions，每个 region 保存其局部结构向量。
    std::vector<Region> regions_;
    // 地图结构向量的统计量，用于归一化不同维度的尺度。
    StructureVector mean_{};
    StructureVector stddev_{};
    bool available_;

    // 在结构兴趣区域周围截取局部三维点云并提取结构向量。
    StructureVector extractAround(double centerX, double centerY, double centerZ, double radius) const;
    // point cloud -> voxelization -> voxel topology graph -> D。
    StructureVector extractFromPoints(const std::vector<Point> &points) const;
    double descriptorDistance2(const StructureVector &a, const StructureVector &b) const;
    void computeDescriptorStatistics(void);

    static StructureVector zeroVector(void);
    static double clamp01(double value);
};

} // namespace mcl3d

#endif // __STRUCTURE_MAP_H__
