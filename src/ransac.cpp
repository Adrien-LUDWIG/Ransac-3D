#include "ransac.h"

#include <Eigen/Geometry>

namespace tnp {

// Ransac for plane detection in 3D
// Normlas should be normalized, otherwise the normal error will be wrong
// because it would not be a cosine distance anymore
std::vector<std::vector<uint>> ransac(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations,
    const std::optional<std::vector<Eigen::Vector3f>>& normals) {
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

      // Check if point is inlier
      bool is_inlier = plane.absDistance(points[i]) <= threshold;

      // If normals are given, check if normal is aligned
      if (normals.has_value())
        is_inlier &= std::abs(plane.normal().dot(normals.value()[i])) > 0.5f;

      if (is_inlier)
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

std::vector<std::vector<Eigen::Vector3f>> ransac_multi(
    const std::vector<Eigen::Vector3f>& points, const float threshold,
    const uint max_number_of_iterations, const uint max_objects,
    const float min_inliers_ratio,
    const std::optional<std::vector<Eigen::Vector3f>>& normals) {
  std::vector<std::vector<Eigen::Vector3f>> objects;
  std::vector<Eigen::Vector3f> remaining_points = points;
  std::optional<std::vector<Eigen::Vector3f>> remaining_normals = normals;

  float inliers_ratio = 1.0;

  while (objects.size() < max_objects && inliers_ratio >= min_inliers_ratio) {
    if (remaining_points.size() == 0) return objects;

    std::vector<std::vector<uint>> indexes =
        ransac(remaining_points, threshold, max_number_of_iterations, remaining_normals);

    std::vector<Eigen::Vector3f> inliers;
    std::vector<Eigen::Vector3f> outliers;
    std::vector<Eigen::Vector3f> outliers_normals = std::vector<Eigen::Vector3f>();

    for (uint i : indexes[0]) {
      inliers.push_back(remaining_points[i]);
    }

    for (uint i : indexes[1]) {
      outliers.push_back(remaining_points[i]);
      if (remaining_normals.has_value()) outliers_normals.push_back(remaining_normals.value()[i]);
    }

    inliers_ratio = float(inliers.size()) / points.size();

    if (inliers_ratio >= min_inliers_ratio) {
      objects.push_back(inliers);
      remaining_points = outliers;
      if (remaining_normals.has_value()) remaining_normals = outliers_normals;
    }
  }

  objects.push_back(remaining_points);
  return objects;
}

}  // namespace tnp