#pragma once

#include <Coordinate.h>
#include <array>
#include <memory>

namespace qh {
using VertexIndex = std::size_t;

struct VertexIndexAndCoordinate {
  VertexIndex index;
  hull::Coordinate coordinates;
};

class PointCloud {
public:
  virtual ~PointCloud() = default;

  virtual std::array<VertexIndexAndCoordinate, 4> getInitialTethraedron() = 0;

  struct FarthestResult {
    VertexIndexAndCoordinate vertex_and_coordinate;
    float distance;
  };
  virtual std::unique_ptr<FarthestResult>
  getFarthest(const hull::Coordinate &point_on_facet,
              const hull::Coordinate &facet_normal) const = 0;

  virtual hull::Coordinate accessVertex(const VertexIndex incidence) const = 0;
  virtual void invalidateVertex(const VertexIndex incidence) = 0;
};
} // namespace qh
