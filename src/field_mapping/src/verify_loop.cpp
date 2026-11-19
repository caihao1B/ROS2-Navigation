#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/SensorData.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using Json = nlohmann::json;

static Json readJson(const char *path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error(std::string("Cannot read ") + path);
    return Json::parse(file);
}

static rtabmap::SensorData makeScan(const Json &points) {
    if (points.empty()) throw std::runtime_error("Empty point cloud");
    cv::Mat cloud(1, static_cast<int>(points.size()), CV_32FC3);
    for (int i = 0; i < cloud.cols; ++i) {
        auto xyz = points.at(i).get<std::array<float, 3>>();
        for (float c : xyz) if (!std::isfinite(c)) throw std::runtime_error("Nonfinite return");
        cloud.at<cv::Vec3f>(0, i) = cv::Vec3f(xyz[0], xyz[1], xyz[2]);
    }
    rtabmap::SensorData data;
    data.setLaserScan(rtabmap::LaserScan(cloud, cloud.cols, 35.f, rtabmap::LaserScan::kXYZ));
    return data;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "Usage: verify_loop OBSERVATIONS.json REFERENCE.json\n";
        return 2;
    }
    try {
        const auto input = readJson(argv[1]);
        const auto heading = input.at("relative_orientation_xyzw").get<std::array<float, 4>>();
        // A proposed revisit at the same position, seeded only with relative odometric attitude.
        const rtabmap::Transform guess(0.f, 0.f, 0.f, heading[0], heading[1], heading[2], heading[3]);
        auto first = makeScan(input.at("first"));
        auto revisit = makeScan(input.at("revisit"));
        const rtabmap::ParametersMap settings = {
            {"Icp/Strategy","1"}, {"Icp/VoxelSize","0.20"}, {"Icp/PointToPlane","true"},
            {"Icp/PointToPlaneK","12"}, {"Icp/PointToPlaneMinComplexity","0.02"},
            {"Icp/PointToPlaneLowComplexityStrategy","0"}, {"Icp/MaxCorrespondenceDistance","0.7"},
            {"Icp/MaxTranslation","1.5"}, {"Icp/MaxRotation","0.5"},
            {"Icp/CorrespondenceRatio","0.30"}, {"Icp/OutlierRatio","0.75"}, {"Icp/Iterations","30"}
        };
        rtabmap::RegistrationIcp registration(settings);
        rtabmap::RegistrationInfo info;
        const auto result = registration.computeTransformation(first, revisit, guess, &info);
        if (result.isNull()) {
            std::cerr << "REJECTED " << info.rejectedMsg << '\n';
            return 1;
        }
        // Only now read the original graph constraint, strictly on the evaluation side.
        const auto reference = readJson(argv[2]);
        const auto p = reference.at("transform_xyz_quat").get<std::array<float, 7>>();
        const rtabmap::Transform archived(p[0],p[1],p[2],p[3],p[4],p[5],p[6]);
        const auto delta = archived.inverse() * result;
        const float angle = delta.getQuaternionf().angularDistance(Eigen::Quaternionf::Identity());
        std::cout << "RTAB-Map accepted the revisit geometry; matched points: " << info.icpCorrespondences
                  << ", offset from archived closure (m, rad): " << delta.getNorm()
                  << ", " << angle << '\n';
        // Agreement for the decimated example, expressed as one mapping voxel and a heading bound.
        return reference.at("accepted").get<bool>() && delta.getNorm() < 0.20f && angle < 0.10f ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
}
