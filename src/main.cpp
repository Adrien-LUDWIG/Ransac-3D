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

  Eigen::Vector3f red = {1.0, 0, 0};
  Eigen::Vector3f green = {0, 1.0, 0};


  std::vector<Eigen::Vector3f> points_color(points.size());

  for (uint i : indexes[0])
    points_color[i] = red;

  for (uint i : indexes[1])
    points_color[i] = green;

  save_obj("../data/road.obj", points, {}, points_color);

  return 0;
}
