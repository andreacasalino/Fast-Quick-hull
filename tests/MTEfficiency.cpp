#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <QuickHull/FastQuickHull.h>

#include <Logger.h>

#include <chrono>
#include <iostream>

using TimeUnit = std::chrono::nanoseconds;

TimeUnit run_convex_hull(const std::vector<Vector3d> &cloud,
                         const std::optional<std::size_t> &threads) {
  auto tic = std::chrono::high_resolution_clock::now();
  qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                  qh::ConvexHullContext{10000, threads});
  return std::chrono::duration_cast<TimeUnit>(
      std::chrono::high_resolution_clock::now() - tic);
}

std::vector<TimeUnit> run_convex_hull_times(
    const std::size_t times, const std::vector<Vector3d> &cloud,
    const std::optional<std::size_t> &threads = std::nullopt) {
  std::vector<TimeUnit> result;
  result.reserve(times);
  for (std::size_t t = 0; t < times; ++t) {
    result.push_back(run_convex_hull(cloud, threads));
  }
  return result;
}

std::ostream &operator<<(std::ostream &s, const std::vector<TimeUnit> &times) {
  for (const auto &time : times) {
    s << " " << time.count();
  }
  return s;
}

TimeUnit total_time(const std::vector<TimeUnit> &times) {
  TimeUnit result = TimeUnit{0};
  for (const auto &time : times) {
    result += time;
  }
  return result;
}

TEST_CASE("Profiling serial vs mt", "[!mayfail]") {
  const std::size_t trials = 10;
  auto cloud = sampleCloud(50000);

  auto serial_times = run_convex_hull_times(trials, cloud);
  std::cout << "Serial times" << serial_times << std::endl;

  auto parallel_times =
      run_convex_hull_times(trials, cloud, std::make_optional<std::size_t>(2));
  std::cout << "Parallel times" << parallel_times << std::endl;

  CHECK(total_time(parallel_times) < total_time(serial_times));
}
