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
}

std::vector<Vector3d> importSTL(const std::string &stlFileName) {
  std::vector<Vector3d> imported_vertices;

  std::ifstream f(stlFileName);
  if (!f.is_open())
    return {}; // invalid file

  std::string line;

  std::size_t k;
  std::getline(f, line);
  float V_temp[3];
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
      V_temp[0] = static_cast<float>(atof(slices[0].c_str()));
      V_temp[1] = static_cast<float>(atof(slices[1].c_str()));
      V_temp[2] = static_cast<float>(atof(slices[2].c_str()));

      auto it_vertices =
          std::find_if(imported_vertices.begin(), imported_vertices.end(),
                       [&V_temp, &toll2](const Vector3d &v) {
                         float distance =
                             (V_temp[0] - v.x()) * (V_temp[0] - v.x());
                         distance += (V_temp[1] - v.y()) * (V_temp[1] - v.y());
                         distance += (V_temp[2] - v.z()) * (V_temp[2] - v.z());
                         return distance < toll2;
                       });
      if (it_vertices == imported_vertices.end()) {
        imported_vertices.emplace_back(V_temp[0], V_temp[1], V_temp[2]);
      }
    }
    std::getline(f, line);
    std::getline(f, line);
  }
  f.close();
  return imported_vertices;
}
