/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "src/Logger.h"
#include <iostream>

std::vector<Vector3d> sampleCloud(const std::size_t size);

int main() {
  // randomly sample a point cloud of a given number of points
  auto cloud = sampleCloud(50);

  // compute the convex hull of the cloud
  auto convex_hull_facets_incidences = qh::convex_hull(
      cloud.begin(), cloud.end(), to_hull_coordinate, qh::ConvexHullContext{});

  // Log the result into a textual file, which can be visualized
  // running the python script Plotter.py
  logConvexhull(convex_hull_facets_incidences, cloud, "Sample01.json");

  std::cout << "call 'python Plotter.py Sample01.json' to see results"
            << std::endl;

  return EXIT_SUCCESS;
}

float sample() {
  return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.f;
}

std::vector<Vector3d> sampleCloud(const std::size_t size) {
  std::vector<Vector3d> result;
  result.reserve(size);
  for (std::size_t k = 0; k < size; ++k) {
    result.emplace_back(sample(), sample(), sample());
  }
  return result;
}
