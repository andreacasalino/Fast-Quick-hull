/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "ImporterSTL.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <math.h>
#include <stdexcept>

constexpr float TOLLERANCE_CLONE = static_cast<float>(1e-4);
constexpr float TOLLERANCE_CLONE_SQUARED = TOLLERANCE_CLONE * TOLLERANCE_CLONE;

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

std::vector<Vector3d> importSTL(const std::string &stlFileName) {
  std::vector<Vector3d> imported_vertices;

  std::ifstream f(stlFileName);
  if (!f.is_open())
    throw std::runtime_error{"Invalid stl file"};

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

std::string getAnimalSTLLocation(const std::string &animalName) {
  return std::string(ANIMALS_FOLDER + animalName + ".stl");
}
