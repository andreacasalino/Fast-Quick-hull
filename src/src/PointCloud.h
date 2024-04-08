#pragma once

#include <Hull/Coordinate.h>

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

namespace qh {
class PointCloud {
public:
  PointCloud(const std::vector<hull::Coordinate> &points);

  std::array<std::size_t, 4> getInitialTethraedron() const;

  struct FarthestVertex {
    std::size_t vertex;
    float distance;
  };
  std::optional<FarthestVertex>
  getFarthest(const hull::Coordinate &point_on_facet,
              const hull::Coordinate &facet_normal) const;

  void closeVertex(std::size_t index) {
    const auto *to_close = &points[index];
    open_set.erase(to_close);
  };

  const std::vector<hull::Coordinate> &points;

private:
  // <vertex , index in points>
  std::unordered_map<const hull::Coordinate *, std::size_t> open_set;
};
} // namespace qh
