#pragma once

#include <Eigen/Core>
#include <iostream>

namespace tnp {
// Ransac for plane detection in 3D
std::pair<std::vector<uint>, std::vector<uint>> ransac(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations);

std::vector<std::vector<Eigen::Vector3f>> ransac_multi(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations, const uint max_objects,
    const float min_inliers_ratio);

float dist_mean_closest_neighbors(
    std::vector<Eigen::Vector3f> cloud, std::vector<uint> points,
    uint base_point, uint k);

}  // namespace tnp