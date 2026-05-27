/****************************************************************************
 * Structure-aware observation prior for mcl3d_ros
 ****************************************************************************/

#include <mcl3d_ros/StructureMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_set>

namespace mcl3d {

namespace {

struct GridKey {
    int x = 0;
    int y = 0;

    bool operator==(const GridKey &other) const {
        return x == other.x && y == other.y;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey &key) const {
        const std::size_t hx = std::hash<int>{}(key.x);
        const std::size_t hy = std::hash<int>{}(key.y);
        return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6) + (hx >> 2));
    }
};

double safeRatio(double numerator, double denominator) {
    if (denominator <= 1.0e-9)
        return 0.0;
    return numerator / denominator;
}

} // namespace

StructureMap::StructureMap(void):
    available_(false)
{
    mean_ = zeroVector();
    stddev_.fill(1.0);
}

void StructureMap::setParameters(const StructureMapParameters &params) {
    params_ = params;
}

bool StructureMap::buildFromMapPoints(const std::vector<Point> &mapPoints) {
    available_ = false;
    mapPoints_ = mapPoints;
    regions_.clear();
    mean_ = zeroVector();
    stddev_.fill(1.0);

    if (mapPoints_.empty() || params_.bevResolution <= 0.0 || params_.voxelSize <= 0.0)
        return false;

    // Step 1: 将全局点云投影到 BEV 栅格，记录 density、max height、height variance。
    std::unordered_map<GridKey, BevCell, GridKeyHash> cells;
    for (const Point &p : mapPoints_) {
        GridKey key{static_cast<int>(std::floor(p.getX() / params_.bevResolution)),
                    static_cast<int>(std::floor(p.getY() / params_.bevResolution))};
        BevCell &cell = cells[key];
        cell.count++;
        cell.zSum += p.getZ();
        cell.z2Sum += p.getZ() * p.getZ();
        cell.maxZ = std::max(cell.maxZ, static_cast<double>(p.getZ()));
        cell.cx = (static_cast<double>(key.x) + 0.5) * params_.bevResolution;
        cell.cy = (static_cast<double>(key.y) + 0.5) * params_.bevResolution;
    }

    int maxCount = 1;
    for (const auto &kv : cells)
        maxCount = std::max(maxCount, kv.second.count);

    // Step 2: 用邻域占据变化、密度梯度和高度复杂度寻找结构变化明显的栅格。
    std::vector<std::pair<GridKey, BevCell>> candidates;
    for (auto &kv : cells) {
        const GridKey &key = kv.first;
        BevCell &cell = kv.second;
        const double meanZ = cell.zSum / static_cast<double>(cell.count);
        const double varZ = std::max(0.0, cell.z2Sum / static_cast<double>(cell.count) - meanZ * meanZ);

        int occupiedNeighbors = 0;
        int transitions = 0;
        double densityGradient = 0.0;
        const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto &dir : dirs) {
            GridKey nk{key.x + dir[0], key.y + dir[1]};
            auto it = cells.find(nk);
            if (it != cells.end()) {
                occupiedNeighbors++;
                densityGradient += std::abs(cell.count - it->second.count) / static_cast<double>(maxCount);
            } else {
                transitions++;
            }
        }

        const double boundary = static_cast<double>(transitions) / 8.0;
        const double opennessChange = std::abs(4.0 - static_cast<double>(occupiedNeighbors)) / 4.0;
        const double densityChange = densityGradient / 8.0;
        const double heightComplexity = clamp01(std::sqrt(varZ) / 2.0);
        cell.score = clamp01(0.35 * boundary + 0.25 * opennessChange + 0.25 * densityChange + 0.15 * heightComplexity);

        if (cell.score >= params_.candidateScoreThreshold)
            candidates.push_back(kv);
    }

    // 先保留高分区域，再用距离抑制避免同一个结构产生大量重复候选。
    std::sort(candidates.begin(), candidates.end(),
        [](const auto &a, const auto &b) { return a.second.score > b.second.score; });

    for (const auto &candidate : candidates) {
        const BevCell &cell = candidate.second;
        bool tooClose = false;
        for (const Region &region : regions_) {
            const double dx = cell.cx - region.x;
            const double dy = cell.cy - region.y;
            if (std::sqrt(dx * dx + dy * dy) < params_.candidateMinDistance) {
                tooClose = true;
                break;
            }
        }
        if (tooClose)
            continue;

        Region region;
        region.x = cell.cx;
        region.y = cell.cy;
        region.z = cell.zSum / static_cast<double>(cell.count);
        region.score = cell.score;
        // Step 3/4: 候选区域局部三维点云 -> voxel topology graph -> D_map(x)。
        region.descriptor = extractAround(region.x, region.y, region.z, params_.localRadius);
        regions_.push_back(region);
    }

    computeDescriptorStatistics();
    available_ = !regions_.empty();
    return available_;
}

bool StructureMap::isAvailable(void) const {
    return available_;
}

int StructureMap::getRegionCount(void) const {
    return static_cast<int>(regions_.size());
}

StructureMap::StructureVector StructureMap::extractFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) const {
    std::vector<Point> points;
    points.reserve(cloud->points.size());
    for (const auto &p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;
        points.emplace_back(p.x, p.y, p.z);
    }
    return extractFromPoints(points);
}

double StructureMap::likelihood(double x, double y, const StructureVector &localDescriptor, double sigma) const {
    if (!available_ || sigma <= 0.0)
        return 1.0;

    // 结构先验不是 descriptor retrieval，这里只取粒子当前位置附近最近的结构区域。
    const Region *best = nullptr;
    double bestD2 = std::numeric_limits<double>::max();
    for (const Region &region : regions_) {
        const double dx = x - region.x;
        const double dy = y - region.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            best = &region;
        }
    }

    if (best == nullptr)
        return 1.0;

    const double maxLookupRadius = std::max(params_.localRadius, params_.candidateMinDistance) * 1.5;
    if (bestD2 > maxLookupRadius * maxLookupRadius)
        return 1.0;

    const double descriptorD2 = descriptorDistance2(localDescriptor, best->descriptor);
    return std::exp(-descriptorD2 / (2.0 * sigma * sigma));
}

StructureMap::StructureVector StructureMap::extractAround(double centerX, double centerY, double, double radius) const {
    std::vector<Point> points;
    points.reserve(512);
    const double r2 = radius * radius;
    for (const Point &p : mapPoints_) {
        const double dx = p.getX() - centerX;
        const double dy = p.getY() - centerY;
        if (dx * dx + dy * dy <= r2)
            points.push_back(p);
    }
    return extractFromPoints(points);
}

StructureMap::StructureVector StructureMap::extractFromPoints(const std::vector<Point> &points) const {
    StructureVector d = zeroVector();
    if (static_cast<int>(points.size()) < params_.minRegionPoints)
        return d;

    double meanX = 0.0, meanY = 0.0, meanZ = 0.0;
    for (const Point &p : points) {
        meanX += p.getX();
        meanY += p.getY();
        meanZ += p.getZ();
    }
    meanX /= static_cast<double>(points.size());
    meanY /= static_cast<double>(points.size());
    meanZ /= static_cast<double>(points.size());

    // 平面协方差用于估计 linearity；voxel 哈希表用于构建拓扑图节点。
    double cxx = 0.0, cxy = 0.0, cyy = 0.0, zVar = 0.0;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxelIndex;
    std::vector<VoxelKey> voxels;
    voxels.reserve(points.size());
    for (const Point &p : points) {
        const double dx = p.getX() - meanX;
        const double dy = p.getY() - meanY;
        const double dz = p.getZ() - meanZ;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
        zVar += dz * dz;

        VoxelKey key{static_cast<int>(std::floor(p.getX() / params_.voxelSize)),
                     static_cast<int>(std::floor(p.getY() / params_.voxelSize)),
                     static_cast<int>(std::floor(p.getZ() / params_.voxelSize))};
        if (voxelIndex.find(key) == voxelIndex.end()) {
            const int idx = static_cast<int>(voxels.size());
            voxelIndex[key] = idx;
            voxels.push_back(key);
        }
    }

    if (voxels.empty())
        return d;

    const double trace = cxx + cyy;
    const double delta = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
    const double lambdaMax = 0.5 * (trace + delta);
    const double lambdaMin = 0.5 * (trace - delta);
    d[0] = clamp01(safeRatio(lambdaMax - lambdaMin, lambdaMax + lambdaMin));

    // 6-neighbor adjacency 构成 voxel topology graph，edge 表示相邻体素连通。
    std::vector<std::vector<int>> adjacency(voxels.size());
    int edgeCount = 0;
    int exposedFaces = 0;
    const int dirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (size_t i = 0; i < voxels.size(); ++i) {
        for (const auto &dir : dirs) {
            VoxelKey nk{voxels[i].x + dir[0], voxels[i].y + dir[1], voxels[i].z + dir[2]};
            auto it = voxelIndex.find(nk);
            if (it == voxelIndex.end()) {
                exposedFaces++;
            } else if (it->second > static_cast<int>(i)) {
                adjacency[i].push_back(it->second);
                adjacency[it->second].push_back(static_cast<int>(i));
                edgeCount++;
            }
        }
    }

    std::vector<int> visited(voxels.size(), 0);
    std::vector<int> componentSizes;
    for (size_t i = 0; i < voxels.size(); ++i) {
        if (visited[i])
            continue;
        int size = 0;
        std::queue<int> q;
        q.push(static_cast<int>(i));
        visited[i] = 1;
        while (!q.empty()) {
            int idx = q.front();
            q.pop();
            size++;
            for (int next : adjacency[idx]) {
                if (!visited[next]) {
                    visited[next] = 1;
                    q.push(next);
                }
            }
        }
        componentSizes.push_back(size);
    }

    const int voxelCount = static_cast<int>(voxels.size());
    const int largestComponent = *std::max_element(componentSizes.begin(), componentSizes.end());
    int branchingNodes = 0;
    for (const auto &neighbors : adjacency) {
        if (neighbors.size() >= 3)
            branchingNodes++;
    }

    // 第一版使用图的连通分量近似 Betti-0，用 cyclomatic number 近似 Betti-1。
    const int betti0 = static_cast<int>(componentSizes.size());
    const int betti1 = std::max(0, edgeCount - voxelCount + betti0);

    double entropy = 0.0;
    for (int size : componentSizes) {
        const double p = static_cast<double>(size) / static_cast<double>(voxelCount);
        if (p > 0.0)
            entropy -= p * std::log(p);
    }
    if (componentSizes.size() > 1)
        entropy /= std::log(static_cast<double>(componentSizes.size()));

    d[1] = clamp01(static_cast<double>(exposedFaces) / (6.0 * static_cast<double>(voxelCount)));
    d[2] = clamp01(static_cast<double>(largestComponent) / static_cast<double>(voxelCount));
    d[3] = clamp01(static_cast<double>(branchingNodes) / static_cast<double>(voxelCount));
    d[4] = clamp01(static_cast<double>(betti0) / std::sqrt(static_cast<double>(voxelCount)));
    d[5] = clamp01(static_cast<double>(betti1) / static_cast<double>(std::max(1, voxelCount)));
    d[6] = clamp01(entropy);
    d[7] = clamp01(0.25 * d[1] + 0.25 * d[3] + 0.25 * d[5] + 0.25 * clamp01(std::sqrt(zVar / points.size()) / 2.0));
    return d;
}

double StructureMap::descriptorDistance2(const StructureVector &a, const StructureVector &b) const {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double scale = std::max(0.05, stddev_[i]);
        const double diff = (a[i] - b[i]) / scale;
        sum += diff * diff;
    }
    return sum / static_cast<double>(a.size());
}

void StructureMap::computeDescriptorStatistics(void) {
    mean_ = zeroVector();
    stddev_.fill(1.0);
    if (regions_.empty())
        return;

    for (const Region &region : regions_) {
        for (size_t i = 0; i < mean_.size(); ++i)
            mean_[i] += region.descriptor[i];
    }
    for (double &value : mean_)
        value /= static_cast<double>(regions_.size());

    stddev_.fill(0.0);
    for (const Region &region : regions_) {
        for (size_t i = 0; i < stddev_.size(); ++i) {
            const double diff = region.descriptor[i] - mean_[i];
            stddev_[i] += diff * diff;
        }
    }
    for (double &value : stddev_)
        value = std::sqrt(value / static_cast<double>(regions_.size()));
}

StructureMap::StructureVector StructureMap::zeroVector(void) {
    StructureVector v{};
    v.fill(0.0);
    return v;
}

double StructureMap::clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

std::size_t StructureMap::VoxelKeyHash::operator()(const VoxelKey &key) const {
    const std::size_t hx = std::hash<int>{}(key.x);
    const std::size_t hy = std::hash<int>{}(key.y);
    const std::size_t hz = std::hash<int>{}(key.z);
    std::size_t seed = hx;
    seed ^= hy + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= hz + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
}

} // namespace mcl3d
