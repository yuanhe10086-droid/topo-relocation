#include <mcl3d_tda_ros/DescriptorDB.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>

namespace mcl3d {

namespace {

double wrapAngle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

std::vector<std::string> splitCSVLine(const std::string &line) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ','))
        fields.push_back(field);
    return fields;
}

} // namespace

bool DescriptorDB::loadCSV(const std::string &filePath) {
    std::ifstream ifs(filePath);
    if (!ifs.is_open()) {
        fprintf(stderr, "Cannot open descriptor DB -> %s\n", filePath.c_str());
        return false;
    }

    entries_.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        std::vector<std::string> fields = splitCSVLine(line);
        if (fields.size() < 7)
            continue;

        DescriptorEntry entry;
        entry.pose.setPose(
            std::stod(fields[0]),
            std::stod(fields[1]),
            std::stod(fields[2]),
            std::stod(fields[3]),
            std::stod(fields[4]),
            std::stod(fields[5]));
        entry.descriptor.reserve(fields.size() - 6);
        for (size_t i = 6; i < fields.size(); ++i)
            entry.descriptor.push_back(std::stod(fields[i]));
        entries_.push_back(entry);
    }

    return !entries_.empty();
}

bool DescriptorDB::saveCSV(const std::string &filePath) const {
    std::ofstream ofs(filePath);
    if (!ofs.is_open()) {
        fprintf(stderr, "Cannot write descriptor DB -> %s\n", filePath.c_str());
        return false;
    }

    ofs << "# x,y,z,roll,pitch,yaw,d0...\n";
    for (const auto &entry : entries_) {
        Pose pose = entry.pose;
        ofs << pose.getX() << "," << pose.getY() << "," << pose.getZ() << ","
            << pose.getRoll() << "," << pose.getPitch() << "," << pose.getYaw();
        for (double value : entry.descriptor)
            ofs << "," << value;
        ofs << "\n";
    }

    return true;
}

void DescriptorDB::clear(void) {
    entries_.clear();
}

void DescriptorDB::addEntry(const Pose &pose, const std::vector<double> &descriptor) {
    entries_.push_back({pose, descriptor});
}

bool DescriptorDB::empty(void) const {
    return entries_.empty();
}

size_t DescriptorDB::size(void) const {
    return entries_.size();
}

std::vector<std::pair<Pose, double>> DescriptorDB::queryTopK(const std::vector<double> &descriptor, int k) const {
    std::vector<std::pair<int, double>> scored;
    scored.reserve(entries_.size());
    for (int i = 0; i < (int)entries_.size(); ++i)
        scored.emplace_back(i, descriptorDistance(descriptor, entries_[i].descriptor));

    std::sort(scored.begin(), scored.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.second < rhs.second;
    });

    std::vector<std::pair<Pose, double>> result;
    int resultNum = std::min(k, (int)scored.size());
    result.reserve(resultNum);
    for (int i = 0; i < resultNum; ++i)
        result.emplace_back(entries_[scored[i].first].pose, scored[i].second);
    return result;
}

double DescriptorDB::descriptorDistanceAtPose(const std::vector<double> &descriptor, Pose pose,
    double xyWeight, double yawWeight) const
{
    if (entries_.empty())
        return std::numeric_limits<double>::infinity();

    int bestIdx = 0;
    double bestPoseDistance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < (int)entries_.size(); ++i) {
        Pose entryPose = entries_[i].pose;
        double dx = pose.getX() - entryPose.getX();
        double dy = pose.getY() - entryPose.getY();
        double dyaw = wrapAngle(pose.getYaw() - entryPose.getYaw());
        double poseDistance = xyWeight * (dx * dx + dy * dy) + yawWeight * dyaw * dyaw;
        if (poseDistance < bestPoseDistance) {
            bestPoseDistance = poseDistance;
            bestIdx = i;
        }
    }

    return descriptorDistance(descriptor, entries_[bestIdx].descriptor);
}

std::vector<double> DescriptorDB::computeHeightLayerDescriptor(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
        Pose centerPose,
    double radius,
    int heightLayers,
    int rangeBins,
    double minZ,
    double maxZ)
{
    heightLayers = std::max(1, heightLayers);
    rangeBins = std::max(1, rangeBins);

    std::vector<double> descriptor(heightLayers * rangeBins, 0.0);
    double yaw = centerPose.getYaw();
    double cy = std::cos(-yaw);
    double sy = std::sin(-yaw);
    double heightSpan = std::max(0.01, maxZ - minZ);

    int usedPoints = 0;
    for (const auto &point : cloud.points) {
        double dx = point.x - centerPose.getX();
        double dy = point.y - centerPose.getY();
        double localX = cy * dx - sy * dy;
        double localY = sy * dx + cy * dy;
        double localZ = point.z - centerPose.getZ();
        double range = std::sqrt(localX * localX + localY * localY);
        if (range > radius || localZ < minZ || localZ > maxZ)
            continue;

        int rangeIdx = std::min(rangeBins - 1, std::max(0, (int)(range / radius * rangeBins)));
        int heightIdx = std::min(heightLayers - 1, std::max(0, (int)((localZ - minZ) / heightSpan * heightLayers)));
        descriptor[heightIdx * rangeBins + rangeIdx] += 1.0;
        usedPoints++;
    }

    if (usedPoints > 0) {
        double norm = 0.0;
        for (double value : descriptor)
            norm += value * value;
        norm = std::sqrt(norm);
        if (norm > 0.0) {
            for (double &value : descriptor)
                value /= norm;
        }
    }

    return descriptor;
}

double DescriptorDB::descriptorDistance(const std::vector<double> &a, const std::vector<double> &b) {
    if (a.empty() || b.empty() || a.size() != b.size())
        return std::numeric_limits<double>::infinity();

    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

} // namespace mcl3d
