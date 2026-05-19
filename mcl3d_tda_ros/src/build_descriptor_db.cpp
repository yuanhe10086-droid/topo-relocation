#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <mcl3d_tda_ros/DescriptorDB.h>

namespace {

void printUsage(void) {
    printf("usage: build_descriptor_db <pcd_file> <descriptor_csv> "
           "<xy_resolution> <yaw_resolution_deg> <radius> <height_layers> <range_bins> <min_z> <max_z>\n");
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 10) {
        printUsage();
        return 1;
    }

    std::string pcdFile = argv[1];
    std::string csvFile = argv[2];
    double xyResolution = std::atof(argv[3]);
    double yawResolution = std::atof(argv[4]) * M_PI / 180.0;
    double radius = std::atof(argv[5]);
    int heightLayers = std::atoi(argv[6]);
    int rangeBins = std::atoi(argv[7]);
    double minZ = std::atof(argv[8]);
    double maxZ = std::atof(argv[9]);

    if (xyResolution <= 0.0 || yawResolution <= 0.0 || radius <= 0.0) {
        fprintf(stderr, "xy_resolution, yaw_resolution_deg, and radius must be positive.\n");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ> mapCloud;
    if (pcl::io::loadPCDFile(pcdFile, mapCloud) != 0) {
        fprintf(stderr, "Cannot read PCD file -> %s\n", pcdFile.c_str());
        return 1;
    }
    if (mapCloud.empty()) {
        fprintf(stderr, "The input map point cloud is empty.\n");
        return 1;
    }

    double minX = mapCloud.points[0].x;
    double maxX = mapCloud.points[0].x;
    double minY = mapCloud.points[0].y;
    double maxY = mapCloud.points[0].y;
    for (const auto &point : mapCloud.points) {
        minX = std::min(minX, (double)point.x);
        maxX = std::max(maxX, (double)point.x);
        minY = std::min(minY, (double)point.y);
        maxY = std::max(maxY, (double)point.y);
    }

    mcl3d::DescriptorDB db;
    int sampledNum = 0;
    int yawNum = std::max(1, (int)std::ceil(2.0 * M_PI / yawResolution));
    for (double x = minX; x <= maxX; x += xyResolution) {
        for (double y = minY; y <= maxY; y += xyResolution) {
            int localPointNum = 0;
            double groundZ = std::numeric_limits<double>::infinity();
            for (const auto &point : mapCloud.points) {
                double dx = point.x - x;
                double dy = point.y - y;
                if (std::sqrt(dx * dx + dy * dy) <= radius) {
                    localPointNum++;
                    groundZ = std::min(groundZ, (double)point.z);
                }
            }
            if (localPointNum < 30 || !std::isfinite(groundZ))
                continue;

            for (int yi = 0; yi < yawNum; ++yi) {
                double yaw = -M_PI + (2.0 * M_PI * (double)yi / (double)yawNum);
                mcl3d::Pose pose(x, y, groundZ, 0.0, 0.0, yaw);
                std::vector<double> descriptor = mcl3d::DescriptorDB::computeHeightLayerDescriptor(
                    mapCloud, pose, radius, heightLayers, rangeBins, minZ, maxZ);
                db.addEntry(pose, descriptor);
                sampledNum++;
            }
        }
    }

    if (!db.saveCSV(csvFile))
        return 1;

    printf("descriptor db saved: %s (%d entries)\n", csvFile.c_str(), sampledNum);
    return 0;
}
