#include "ransac.h"

#include <Eigen/Geometry>

namespace tnp {

std::vector<std::vector<uint>> ransac(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations) {
  std::vector<uint> best_inliers;
  std::vector<uint> best_outliers;

  for (uint k = 0; k < max_number_of_iterations; k++) {
    uint a = std::rand() % points.size();
    uint b = std::rand() % points.size();
    uint c = std::rand() % points.size();

    // Create Eigen plane
    Eigen::Hyperplane<float, 3> plane =
        Eigen::Hyperplane<float, 3>::Through(points[a], points[b], points[c]);

    std::vector<uint> inliers;
    std::vector<uint> outliers;

    // Find inliers
    for (uint i = 0; i < points.size(); i++) {
      if (plane.absDistance(points[i]) <= threshold)
        inliers.push_back(i);
      else
        outliers.push_back(i);
    }

    // Check if new plane has more inliers
    if (inliers.size() > best_inliers.size()) {
      best_inliers = inliers;
      best_outliers = outliers;
    }
  }

  return {best_inliers, best_outliers};
}

}  // namespace tnp