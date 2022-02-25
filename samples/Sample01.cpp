/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Logger.h>
#include <iostream>

int main() {
  // randomly sample a point cloud of a given number of points
  auto cloud = sampleCloud(50);

  // compute the convex hull of the cloud
  std::vector<hull::Coordinate> convex_hull_normals;
  auto convex_hull_facets_incidences =
      qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                      convex_hull_normals, qh::ConvexHullContext{});

  // Log the result into a textual file, which can be visualized
  // running the python script Plotter.py
  logConvexhull(convex_hull_facets_incidences, convex_hull_normals, cloud,
                "Sample01.json");

  std::cout << "call 'python Plotter.py Sample01.json' to see results"
            << std::endl;

  return EXIT_SUCCESS;
}
