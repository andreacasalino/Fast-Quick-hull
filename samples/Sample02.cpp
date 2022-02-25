/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ImporterSTL.h>
#include <iostream>

int main() {
  const std::size_t thread_pool_size =
      1; // setting 1 is like not using at all the thread pool, 0 exploits all
         // possible threads, a different number specify the exact number of
         // threads to use

  // import the cloud from existing STL files
  std::vector<std::string> stlNames;
  stlNames.push_back("Dolphin"); // 672 vertices
  stlNames.push_back("Giraffe"); // 788 vertices
  stlNames.push_back("Hyppo");   // 2399 vertices
  stlNames.push_back("Snake");   // 3691 vertices
  stlNames.push_back("Eagle");   // 4271 vertices

  for (auto it = stlNames.begin(); it != stlNames.end(); ++it) {
    std::cout << "computing convex hull of " << *it;

    // import the stl describing the shape of this animal
    const std::string stl_location = getAnimalSTLLocation(*it);
    const auto vertices_cloud = importSTL(stl_location);

    // compute the convex hull of of the imported vertices cloud
    std::vector<hull::Coordinate> convex_hull_normals;
    auto convex_hull_facets_incidences = qh::convex_hull(
        vertices_cloud.begin(), vertices_cloud.end(), to_hull_coordinate,
        convex_hull_normals, qh::ConvexHullContext{2000, thread_pool_size});

    // Log the result into a textual file, which can be visualized
    // running the python script Plotter.py
    logConvexhull(convex_hull_facets_incidences, convex_hull_normals,
                  vertices_cloud, *it + ".json");
    std::cout << " done" << std::endl;
    std::cout << "call 'python Plotter.py " << *it << ".json " << stl_location
              << "' to see results" << std::endl
              << std::endl;
  }

  return EXIT_SUCCESS;
}