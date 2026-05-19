#ifndef __DESCRIPTOR_DB_H__
#define __DESCRIPTOR_DB_H__

#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mcl3d_tda_ros/Pose.h>

namespace mcl3d {

struct DescriptorEntry {
    Pose pose;
    std::vector<double> descriptor;
};

class DescriptorDB {
private:
    std::vector<DescriptorEntry> entries_;

public:
    bool loadCSV(const std::string &filePath);
    bool saveCSV(const std::string &filePath) const;
    void clear(void);
    void addEntry(const Pose &pose, const std::vector<double> &descriptor);
    bool empty(void) const;
    size_t size(void) const;

    std::vector<std::pair<Pose, double>> queryTopK(const std::vector<double> &descriptor, int k) const;
    double descriptorDistanceAtPose(const std::vector<double> &descriptor, Pose pose,
        double xyWeight, double yawWeight) const;

    static std::vector<double> computeHeightLayerDescriptor(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        Pose centerPose,
        double radius,
        int heightLayers,
        int rangeBins,
        double minZ,
        double maxZ);

    static double descriptorDistance(const std::vector<double> &a, const std::vector<double> &b);
};

} // namespace mcl3d

#endif // __DESCRIPTOR_DB_H__
