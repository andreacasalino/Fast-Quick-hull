#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <QuickHull/FastQuickHull.h>

#include <Logger.h>

TEST_CASE("Random clouds") {
  auto cloud = sampleCloud(2000);
  auto samples = GENERATE(10, 50, 100, 200, 500, 1000, 2000);

  auto cloud_it_end = cloud.begin();
  std::advance(cloud_it_end, samples);

  SECTION("Serial") {
    qh::convex_hull(cloud.begin(), cloud_it_end, to_hull_coordinate,
                    qh::ConvexHullContext{2000, std::nullopt});
  }

  SECTION("Multi Threaded") {
    qh::convex_hull(cloud.begin(), cloud_it_end, to_hull_coordinate,
                    qh::ConvexHullContext{2000, 2});
  }
}
