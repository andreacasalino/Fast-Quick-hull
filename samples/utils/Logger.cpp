/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Logger.h"
#include <algorithm>
#include <fstream>
#include <random>

hull::Coordinate to_hull_coordinate(const Vector3d &to_convert) {
  return hull::Coordinate{to_convert.x(), to_convert.y(), to_convert.z()};
}

template <typename T, typename ValueExtractor>
void append_vector(std::ofstream &stream, const std::vector<T> &collection,
                   const ValueExtractor &extractor) {
  auto append_element = [&stream, &extractor](const T &element) {
    stream << '[' << std::to_string(extractor(element, 0)) << ','
           << std::to_string(extractor(element, 1)) << ','
           << std::to_string(extractor(element, 2)) << ']' << std::endl;
  };

  stream << '[' << std::endl;
  if (!collection.empty()) {
    auto collection_it = collection.begin();
    append_element(*collection_it);
    ++collection_it;
    std::for_each(collection_it, collection.end(),
                  [&append_element, &stream](const T &element) {
                    stream << ',';
                    append_element(element);
                  });
  }
  stream << ']' << std::endl;
}

void logConvexhull(const std::vector<qh::FacetIncidences> &facets_incidences,
                   const std::vector<hull::Coordinate> &convex_hull_normals,
                   const std::vector<Vector3d> &cloud,
                   const std::string &fileName) {
  std::ofstream f(fileName);
  if (!f.is_open())
    return;

  f << '{' << std::endl;
  f << "\"Cloud\":";
  append_vector(f, cloud, [](const Vector3d &element, const std::size_t index) {
    switch (index) {
    case 0:
      return element.x();
    case 1:
      return element.y();
    default:
      break;
    }
    return element.z();
  });
  f << ",\"Index\":";
  append_vector(f, facets_incidences,
                [](const qh::FacetIncidences &element,
                   const std::size_t index) { return element[index]; });
  f << ",\"Normals\":";
  append_vector(f, convex_hull_normals,
                [](const hull::Coordinate &element, const std::size_t index) {
                  switch (index) {
                  case 0:
                    return element.x;
                  case 1:
                    return element.y;
                  default:
                    break;
                  }
                  return element.z;
                });
  f << '}';
}

float sample() {
  return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.f;
}

static bool reset_seed = true;

std::vector<Vector3d> sampleCloud(const std::size_t size) {
  if (reset_seed) {
    srand(0);
    reset_seed = false;
  }
  std::vector<Vector3d> result;
  result.reserve(size);
  for (std::size_t k = 0; k < size; ++k) {
    result.emplace_back(sample(), sample(), sample());
  }
  return result;
}
