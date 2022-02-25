#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <QuickHull/FastQuickHull.h>

#include <memory>

std::vector<hull::Coordinate> import_STL(const std::string &fileName) {
  std::string path = ANIMALS_FOLDER + fileName + ".stl";
  // TODO
};

TEST_CASE("Random clouds") {
  auto file_name = GENERATE("Dolphin", "Eagle", "Giraffe", "Hyppo", "Snake");

  auto animal = import_STL(file_name);

  qh::convex_hull(animal, qh::ConvexHullContext{2000, std::nullopt});
}
