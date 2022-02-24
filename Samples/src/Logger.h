/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <QuickHull/FastQuickHull.h>

#include <string>
#include <vector>

/** @brief Just an example of 3d coordinate representation: you can use the one
 * defined in your favourite linear algebra library (Eigen, etc...)
 */
class Vector3d {
public:
  Vector3d(const float &x, const float &y, const float &z) {
    this->coordinates[0] = x;
    this->coordinates[1] = y;
    this->coordinates[2] = z;
  };

  inline float x() const { return this->coordinates[0]; };
  inline float y() const { return this->coordinates[1]; };
  inline float z() const { return this->coordinates[2]; };

private:
  float coordinates[3];
};

hull::Coordinate to_hull_coordinate(const Vector3d &to_convert);

void logConvexhull(const std::vector<qh::FacetIncidences> &facets_incidences,
                   const std::vector<Vector3d> &cloud,
                   const std::string &fileName);
