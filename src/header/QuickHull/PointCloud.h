#pragma once

#include <Hull/Coordinate.h>
#include <memory>
#include <vector>

namespace qh {
struct VertexAndCoordinate {
  std::size_t vertex_index;
  hull::Coordinate coordinate;
};

class PointCloud {
public:
  PointCloud(const std::vector<hull::Coordinate> &points);

  std::vector<VertexAndCoordinate> getInitialTethraedron() const;

  struct FarthestVertex {
    VertexAndCoordinate vertex_and_coordinate;
    float distance;
  };
  std::unique_ptr<FarthestVertex>
  getFarthest(const hull::Coordinate &point_on_facet,
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
