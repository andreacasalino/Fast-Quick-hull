#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <ImporterSTL.h>

TEST_CASE("Animals STL") {
  auto animal_name = GENERATE("Dolphin", "Eagle", "Giraffe", "Hyppo", "Snake");

  auto cloud = importSTL(ANIMALS_FOLDER + std::string{animal_name} + ".stl");

  qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                  qh::ConvexHullContext{2000, std::nullopt});
}
