#include <kdtree.h>
#include <obj.h>

#include "ransac.h"

using namespace tnp;

std::vector<Eigen::Vector3f> COLORS{{255. / 255., 179. / 255., 0. / 255.},
                                    {128. / 255., 62. / 255., 117. / 255.},
                                    {255. / 255., 104. / 255., 0. / 255.},
                                    {166. / 255., 189. / 255., 215. / 255.},
                                    {193. / 255., 0. / 255., 32. / 255.},
                                    {206. / 255., 162. / 255., 98. / 255.},
                                    {129. / 255., 112. / 255., 102. / 255.},
                                    {0. / 255., 125. / 255., 52. / 255.},
                                    {246. / 255., 118. / 255., 142. / 255.},
                                    {0. / 255., 83. / 255., 138. / 255.},
                                    {255. / 255., 122. / 255., 92. / 255.},
                                    {83 / 255, 55. / 255., 122. / 255.},
                                    {255. / 255., 142. / 255., 0. / 255.},
                                    {179. / 255., 40. / 255., 81. / 255.},
                                    {244. / 255., 200. / 255., 0. / 255.},
                                    {127. / 255., 24. / 255., 13. / 255.},
                                    {147. / 255., 170. / 255., 0. / 255.},
                                    {89. / 255., 51. / 255., 21. / 255.},
                                    {241. / 255., 58. / 255., 19. / 255.},
                                    {35. / 255., 44. / 255., 22. / 255.}};

void coloring_and_save(std::string filename,
                       std::vector<std::vector<Eigen::Vector3f>> objects) {
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> colors;

  uint color_idx = 0;
  for (std::vector<Eigen::Vector3f> object : objects) {
    Eigen::Vector3f color = COLORS[color_idx];
    for (Eigen::Vector3f point : object) {
      points.push_back(point);
      colors.push_back(color);
    }
    color_idx = (color_idx + 1) % COLORS.size();
  }

  save_obj(filename, points, {}, colors);

  std::cout << "Saved " << objects.size() << " objects." << std::endl;
}

int main(int argc, char* argv[]) {
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
  const float threshold = 0.25;
  const uint max_number_of_iterations = 1000;

  int max_objects = 5;
  if (argc == 3)
    max_objects = std::stoi(argv[2]);

  float min_inliers_ratio = 0.05;
  if (argc == 4)
    min_inliers_ratio = std::stof(argv[3]);

  std::vector<std::vector<Eigen::Vector3f>> objects =
      ransac_multi(points, threshold, max_number_of_iterations, max_objects,
                   min_inliers_ratio);

  coloring_and_save("../data/multi_ransac.obj", objects);

  return 0;
}
