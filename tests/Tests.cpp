#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <Utils.h>

TEST_CASE("Random clouds") {
  auto cloud = sampleCloud(2000);
  auto samples = GENERATE(10, 50, 100, 200, 500, 1000, 2000);

  SECTION("Serial") {
    qh::convex_hull(cloud.begin(), cloud.begin() + samples, to_hull_coordinate,
                    qh::ConvexHullContext{2000, std::nullopt});
  }

  SECTION("Multi Threaded") {
    qh::convex_hull(cloud.begin(), cloud.begin() + samples, to_hull_coordinate,
                    qh::ConvexHullContext{2000, 2});
  }
}

TEST_CASE("Animals STL") {
  auto animal_name = GENERATE("Dolphin", "Eagle", "Giraffe", "Hyppo", "Snake");

  auto cloud = importAnimalStl(animal_name);

  qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                  qh::ConvexHullContext{2000, std::nullopt});
}

/////////////////////////////////////////////////////////
/////////////////// performance tests ///////////////////
/////////////////////////////////////////////////////////

#include <chrono>
#include <iostream>

struct Durations {
  using TimeUnit = std::chrono::nanoseconds;

  Durations() = default;

  template <typename Pred> Durations &add(const Pred &pred) {
    auto tic = std::chrono::high_resolution_clock::now();
    pred();
    auto elapsed = std::chrono::duration_cast<TimeUnit>(
        std::chrono::high_resolution_clock::now() - tic);
    auto it = std::upper_bound(samples.begin(), samples.end(), elapsed);
    samples.insert(it, elapsed);
    return *this;
  }

  template <typename Pred>
  Durations &addXTimes(const Pred &pred, std::size_t times) {
    for (std::size_t k = 0; k < times; ++k) {
      add(pred);
    }
    return *this;
  }

  struct Stats {
    TimeUnit average;
    TimeUnit min;
    TimeUnit max;
  };
  Stats getStats() const {
    Stats res;
    res.min = samples.front();
    res.max = samples.back();
    double average = 0;
    for (const auto &sample : samples) {
      average += static_cast<double>(sample.count()) /
                 static_cast<double>(samples.size());
    }
    res.average = TimeUnit{static_cast<int>(round(average))};
    return res;
  }

private:
  std::vector<TimeUnit> samples;
};

std::ostream &operator<<(std::ostream &recipient,
                         const Durations::Stats &stats) {
  auto convert_time = [](const Durations::TimeUnit &t) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(t);
  };

  return recipient << "[ " << convert_time(stats.min).count() << " , "
                   << convert_time(stats.max).count() << " ] (ms)  average "
                   << convert_time(stats.average).count() << " (ms)";
}

TEST_CASE("Profiling serial vs mt", "[!mayfail]") {
  const std::size_t trials = 10;
  auto cloud = sampleCloud(50000);

  Durations::Stats serial_times =
      Durations{}
          .addXTimes(
              [&]() {
                qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                                qh::ConvexHullContext{10000, std::nullopt});
              },
              trials)
          .getStats();
  std::cout << "Serial         " << serial_times << std::endl;

  Durations::Stats mt_times =
      Durations{}
          .addXTimes(
              [&]() {
                qh::convex_hull(cloud.begin(), cloud.end(), to_hull_coordinate,
                                qh::ConvexHullContext{
                                    10000, std::make_optional<std::size_t>(2)});
              },
              trials)
          .getStats();
  std::cout << "Multi Threaded " << serial_times << std::endl;

  CHECK(serial_times.average < mt_times.average);
}