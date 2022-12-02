#include <kdtree.h>
#include <obj.h>

#include "ransac.h"

using namespace tnp;

int main(int argc, char *argv[]) {
  // option -----------------------------------------------------------------
  if (argc <= 1) {
    std::cout << "Error: missing filename" << std::endl;
    return 1;
  }
  const auto filename = std::string(argv[1]);

  // load -------------------------------------------------------------------
  auto points = std::vector<Eigen::Vector3f>();
  auto normals = std::vector<Eigen::Vector3f>();
  if (not tnp::load_obj(filename, points, normals)) {
    std::cout << "Error: failed to open input file '" << filename << "'"
              << std::endl;
    return 1;
  }

  // process ----------------------------------------------------------------  
  const float threshold = 0.5;
  const uint max_number_of_iterations = 1000;
  std::vector<std::vector<uint>> indexes = ransac(points, threshold, max_number_of_iterations);

  // std::vector<uint> inliers = indexes[0];
  // std::vector<uint> outliers = indexes[1];

  std::vector<Eigen::Vector3f> inliers;
  std::vector<Eigen::Vector3f> outliers;

  for (uint i : indexes[0])
    inliers.push_back(points[i]);

  for (uint i : indexes[1])
    outliers.push_back(points[i]);

  save_obj("../data/road.obj", inliers, {});

  return 0;
}
