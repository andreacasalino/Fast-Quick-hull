#pragma once

#include <Hull/Coordinate.h>
#include <vector>

namespace qh {
class PointCloud {
public:
  PointCloud(const std::vector<hull::Coordinate> &points);

  std::vector<std::size_t> getInitialTethraedron() const;

  struct FarthestVertex {
    std::size_t vertex;
    float distance;
  };
  FarthestVertex getFarthest(const hull::Coordinate &point_on_facet,
                             const hull::Coordinate &facet_normal) const;

  const hull::Coordinate &accessVertex(const std::size_t index) const {
    return points[index];
  }
  void invalidateVertex(const std::size_t index) {
    points_still_free[index] = false;
  };

private:
  const std::vector<hull::Coordinate> points;
  std::vector<bool> points_still_free;
};
} // namespace qh
