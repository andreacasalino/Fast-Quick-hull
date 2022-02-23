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
  PointCloud::FarthestVertex result = PointCloud::FarthestVertex{0, 0.f};
  float distance;
  for (std::size_t pos = 0; pos < points.size(); ++pos) {
    if (points_still_free[pos]) {
      distance = get_vertex_distance(point_on_facet, facet_normal, points[pos]);
      if ((QHULL_GEOMETRIC_TOLLERANCE < distance) &&
          (result.distance < distance)) {
        result.distance = distance;
        result.vertex = pos;
      }
    }
  }
  if (result.distance <= QHULL_GEOMETRIC_TOLLERANCE) {
    return nullptr;
  }
  return std::make_unique<PointCloud::FarthestVertex>(result);
}

namespace {
template <typename DistanceComputation>
std::size_t
farthest_to_subject(const std::vector<hull::Coordinate> &points,
                    const DistanceComputation &distance_to_subject) {
  float max_distance = 0.f;
  std::size_t result = 0;
  float distance;
  for (std::size_t k = 0; k < points.size(); ++k) {
    distance = distance_to_subject(points[k]);
    if ((distance > QHULL_GEOMETRIC_TOLLERANCE) && (distance > max_distance)) {
      result = k;
      max_distance = distance;
    }
  }
  if (max_distance <= QHULL_GEOMETRIC_TOLLERANCE) {
    throw Error{"The passed cloud has 0 volume"};
  }
}
} // namespace

std::vector<std::size_t> PointCloud::getInitialTethraedron() const {
  std::vector<std::size_t> result;
  result.reserve(4);

  result.push_back(0);
  const auto first_vertex = result.back();

  result.push_back(farthest_to_subject(
      points, [&subject = points[first_vertex]](const hull::Coordinate &point) {
        return hull::squaredDistance(subject, point);
      }));
  const auto second_vertex = result.back();

  // TODO

  return result;
}
} // namespace qh
