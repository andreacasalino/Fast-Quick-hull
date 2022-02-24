#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <QuickHull/FastQuickHull.h>

#include <memory>

float make_random_coordinate() {
  return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.f;
}

std::vector<hull::Coordinate> make_cloud(const std::size_t samples) {
  std::vector<hull::Coordinate> result;
  result.reserve(samples);
  for (std::size_t k = 0; k < samples; ++k) {
    auto &point = result.emplace_back();
    point.x = make_random_coordinate();
    point.y = make_random_coordinate();
    point.z = make_random_coordinate();
  }
  return result;
}

TEST_CASE("Random clouds") {
  auto cloud = make_cloud(2000);
  auto samples = GENERATE(10, 50, 100, 200, 500, 1000, 2000);

  auto cloud_it_end = cloud.begin();
  std::advance(cloud_it_end, samples);

  SECTION("Serial") {
    qh::convex_hull(std::vector<hull::Coordinate>{cloud.begin(), cloud_it_end},
                    qh::ConvexHullContext{2000, std::nullopt});
  }

  // SECTION("Multi Threaded") {
  //   qh::convex_hull(std::vector<hull::Coordinate>{cloud.begin(),
  //   cloud_it_end},
  //                   qh::ConvexHullContext{2000, 2});
  // }
}
