#pragma once

#include <Eigen/Core>
#include <iostream>

namespace tnp {
// Ransac for plane detection in 3D
std::vector<std::vector<uint>> ransac(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations);

std::vector<std::vector<Eigen::Vector3f>> ransac_multi(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations, const uint nb_objects);

}  // namespace tnp