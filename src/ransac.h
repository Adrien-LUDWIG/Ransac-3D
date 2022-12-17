#pragma once

#include <Eigen/Core>
#include <iostream>
#include <optional>

namespace tnp {
// Ransac for plane detection in 3D
std::pair<std::vector<uint>, std::vector<uint>> ransac(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations,
    const std::optional<std::vector<Eigen::Vector3f>>& normals = std::nullopt,
    bool remove_outliers = false);

std::vector<std::vector<Eigen::Vector3f>> ransac_multi(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations, const uint max_objects,
    const float min_inliers_ratio,
    const std::optional<std::vector<Eigen::Vector3f>>& normals = std::nullopt,
    bool remove_outliers = false);
}  // namespace tnp