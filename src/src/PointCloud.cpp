#include "PointCloud.h"
#include <QuickHull/Error.h>

namespace qh {
namespace {
constexpr float QHULL_GEOMETRIC_TOLLERANCE = static_cast<float>(1e-3);
}

PointCloud::PointCloud(const std::vector<hull::Coordinate> &points)
    : points(points) {
  if (points.size() < 4) {
    throw Error{"The point cloud should have at least 4 points"};
  }
  points_still_free = std::vector<bool>(points.size(), true);
}

namespace {
float get_vertex_distance(const hull::Coordinate &facet_point,
                          const hull::Coordinate &facet_normal,
                          const hull::Coordinate &vertex) {
  hull::Coordinate diff;
  hull::diff(diff, vertex, facet_point);
  return hull::dot(diff, facet_normal);
}
} // namespace

std::unique_ptr<PointCloud::FarthestVertex>
PointCloud::getFarthest(const hull::Coordinate &point_on_facet,
                        const hull::Coordinate &facet_normal) const {
  bool found_at_least_one = false;
  std::unique_ptr<PointCloud::FarthestVertex> result =
      std::make_unique<PointCloud::FarthestVertex>(0, 0.f);
  for (std::size_t pos = 0; pos < points.size(); ++pos) {
    if (points_still_free[pos]) {
      auto distance =
          get_vertex_distance(point_on_facet, facet_normal, points[pos]);
      if ((QHULL_GEOMETRIC_TOLLERANCE < distance) &&
          (result->distance < distance)) {
        result->distance = distance;
        result->vertex = pos;
        found_at_least_one = true;
      }
    }
  }
  if (!found_at_least_one) {
    return nullptr;
  }
  return result;
}
} // namespace qh
