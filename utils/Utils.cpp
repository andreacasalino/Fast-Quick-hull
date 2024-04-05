/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"

#include <algorithm>
#include <fstream>
#include <math.h>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>

hull::Coordinate to_hull_coordinate(const Vector3d &to_convert) {
  return hull::Coordinate{to_convert.x(), to_convert.y(), to_convert.z()};
}

namespace {
std::once_flag reset_seed_flag;
} // namespace

std::vector<Vector3d> sampleCloud(const std::size_t size) {
  std::call_once(reset_seed_flag, []() { srand(0); });
  std::vector<Vector3d> result;
  result.reserve(size);
  auto sample = []() {
    return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) -
           1.f;
  };
  for (std::size_t k = 0; k < size; ++k) {
    result.emplace_back(sample(), sample(), sample());
  }
  return result;
}

namespace {
template <typename T, typename ValueExtractor>
void append_vector(std::ofstream &stream, const std::vector<T> &collection,
                   const ValueExtractor &extractor) {
  auto append_element = [&stream, &extractor](const T &element) {
    stream << '[' << extractor(element, 0) << ',' << extractor(element, 1)
           << ',' << extractor(element, 2) << "]\n";
  };

  stream << "[\n";
  if (!collection.empty()) {
    append_element(collection.front());
    std::for_each(collection.begin() + 1, collection.end(),
                  [&](const T &element) {
                    stream << ',';
                    append_element(element);
                  });
  }
  stream << "]\n";
}
} // namespace

void logConvexhull(const std::vector<qh::FacetIncidences> &facets_incidences,
                   const std::vector<hull::Coordinate> &convex_hull_normals,
                   const std::vector<Vector3d> &cloud,
                   const std::string &fileName) {
  std::ofstream f(fileName);
  if (!f.is_open()) {
    throw std::runtime_error{fileName + " is an invalid filename"};
  }

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

namespace {
static constexpr float TOLLERANCE_CLONE = static_cast<float>(1e-4);
static constexpr float TOLLERANCE_CLONE_SQUARED =
    TOLLERANCE_CLONE * TOLLERANCE_CLONE;

std::vector<std::string> splitta_riga(const std::string &riga) {
  std::istringstream iss(riga);
  std::vector<std::string> slices;
  while (!iss.eof()) {
    iss >> slices.emplace_back();
  }
  return slices;
}

float squared_distance(const Vector3d &a, const Vector3d &b) {
  float distance = std::pow(a.x() - b.x(), 2.f);
  distance += std::pow(a.y() - b.y(), 2.f);
  distance += std::pow(a.z() - b.z(), 2.f);
  return distance;
}
} // namespace

std::vector<Vector3d> importAnimalStl(const std::string &animalName) {
  std::vector<Vector3d> imported_vertices;

  auto filename = getAnimalStlPath(animalName);

  std::ifstream f(filename);
  if (!f.is_open())
    throw std::runtime_error{filename.string() + " is an invalid filename"};

  while (!f.eof()) {
    std::string line;
    std::getline(f, line);
    auto slices = splitta_riga(line);
    if ((!slices.empty()) && (slices.front() == "vertex")) {
      if (slices.size() != 4) {
        throw std::runtime_error{"Invalid line inside stl file"};
      }
      Vector3d maybe_new_vertex =
          Vector3d{static_cast<float>(atof(slices[1].c_str())),
                   static_cast<float>(atof(slices[2].c_str())),
                   static_cast<float>(atof(slices[3].c_str()))};

      auto vertices_clone_it =
          std::find_if(imported_vertices.begin(), imported_vertices.end(),
                       [&maybe_new_vertex](const Vector3d &v) {
                         return squared_distance(maybe_new_vertex, v) <=
                                TOLLERANCE_CLONE_SQUARED;
                       });
      if (vertices_clone_it == imported_vertices.end()) {
        imported_vertices.push_back(maybe_new_vertex);
      }
    }
  }
  return imported_vertices;
}

std::filesystem::path getAnimalStlPath(const std::string &animalName) {
  static std::filesystem::path parent = std::filesystem::path{ANIMALS_FOLDER};
  return parent / std::string{animalName + ".stl"};
}
