/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "src/ImporterSTL.h"
#include <iostream>

int main() {
  const std::size_t thread_pool_size = 1; // TODO explain

  // import the cloud from existing STL files
  std::vector<std::string> stlNames;
  stlNames.push_back("Dolphin"); // 672 vertices
  stlNames.push_back("Giraffe"); // 788 vertices
  stlNames.push_back("Hyppo");   // 2399 vertices
  stlNames.push_back("Snake");   // 3691 vertices
  stlNames.push_back("Eagle");   // 4271 vertices

  for (auto it = stlNames.begin(); it != stlNames.end(); ++it) {
    // import the stl and compute the convex hull. Then, log the results
    // You can use the python script Plotter.py to display the results
    std::cout << "computing convex hull of " << *it;
    const auto vertices_cloud = importSTL("Animals/" + *it + ".stl");
    // compute the convex hull of of the imported vertices cloud
    std::vector<hull::Coordinate> convex_hull_normals;
    auto convex_hull_facets_incidences = qh::convex_hull(
        vertices_cloud.begin(), vertices_cloud.end(), to_hull_coordinate,
        qh::ConvexHullContext{2000, thread_pool_size}, convex_hull_normals);
    // log results
    logConvexhull(convex_hull_facets_incidences, convex_hull_normals,
                  vertices_cloud, *it + ".json");
    std::cout << " done" << std::endl;
    std::cout << "call 'python Plotter.py " << *it << ".json' to see results"
              << std::endl
              << std::endl;
  }

  return EXIT_SUCCESS;
}