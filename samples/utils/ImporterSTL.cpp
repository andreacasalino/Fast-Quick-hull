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

constexpr float TOLLERANCE_CLONE = static_cast<float>(1e-4);

std::vector<std::string> splitta_riga(const std::string &riga) {
  std::istringstream iss(riga);
  std::vector<std::string> slices;
  while (true) {
    if (iss.eof()) {
      break;
    }
    slices.emplace_back();
    iss >> slices.back();
  }
  return slices;
}

std::vector<Vector3d> importSTL(const std::string &stlFileName) {
  std::vector<Vector3d> imported_vertices;

  std::ifstream f(stlFileName);
  if (!f.is_open())
    return {}; // invalid file

  std::string line;

  std::size_t k;
  std::getline(f, line);
  float toll2 = TOLLERANCE_CLONE * TOLLERANCE_CLONE;
  while (true) {
    std::getline(f, line);
    auto slices = splitta_riga(line);
    if (slices.front().compare("endsolid") == 0)
      break;

    std::getline(f, line);
    for (k = 0; k < 3; k++) {
      std::getline(f, line);
      slices = splitta_riga(line);
      float x = static_cast<float>(atof(slices[1].c_str()));
      float y = static_cast<float>(atof(slices[2].c_str()));
      float z = static_cast<float>(atof(slices[3].c_str()));

      auto it_vertices =
          std::find_if(imported_vertices.begin(), imported_vertices.end(),
                       [&](const Vector3d &v) {
                         float distance = (x - v.x()) * (y - v.x());
                         distance += (y - v.y()) * (y - v.y());
                         distance += (z - v.z()) * (z - v.z());
                         return distance < toll2;
                       });
      if (it_vertices == imported_vertices.end()) {
        imported_vertices.emplace_back(x, y, z);
      }
    }
    std::getline(f, line);
    std::getline(f, line);
  }
  f.close();
  return imported_vertices;
}

std::vector<Vector3d> importAnimalSTL(const std::string &animalName) {
  return importSTL(ANIMALS_FOLDER + animalName + ".stl");
}
