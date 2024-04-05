#include "PointCloud.h"
#include <QuickHull/Error.h>

#include "Definitions.h"

namespace qh {
PointCloud::PointCloud(const std::vector<hull::Coordinate> &points)
    : points(points) {
  if (points.size() < 4) {
    throw Error{"The point cloud should have at least 4 points"};
  }
  for (std::size_t k = 0; k < points.size(); ++k) {
    open_set[&points[k]] = k;
  }
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

std::optional<PointCloud::FarthestVertex>
PointCloud::getFarthest(const hull::Coordinate &point_on_facet,
                        const hull::Coordinate &facet_normal) const {
  PointCloud::FarthestVertex result =
      PointCloud::FarthestVertex{0, QHULL_GEOMETRIC_TOLLERANCE};
  float distance;
  for (auto [point, pos] : open_set) {
    distance = get_vertex_distance(point_on_facet, facet_normal, *point);
    if (result.distance < distance) {
      result.distance = distance;
      result.vertex = pos;
    }
  }
  if (result.distance == QHULL_GEOMETRIC_TOLLERANCE) {
    return std::nullopt;
  }
  return result;
}

namespace {
template <typename DistanceComputation>
std::size_t
farthest_to_subject(const std::vector<hull::Coordinate> &points,
                    const DistanceComputation &squared_distance_to_subject) {
  float max_distance = QHULL_GEOMETRIC_TOLLERANCE_SQUARED;
  std::size_t result = 0;
  float squared_distance;
  for (std::size_t k = 0; k < points.size(); ++k) {
    squared_distance = squared_distance_to_subject(points[k]);
    if (max_distance < squared_distance) {
      result = k;
      max_distance = squared_distance;
    }
  }
  if (max_distance == QHULL_GEOMETRIC_TOLLERANCE_SQUARED) {
    throw Error{"The passed cloud has null volume"};
  }
  return result;
}

hull::Coordinate delta(const hull::Coordinate &v1, const hull::Coordinate &v2) {
  hull::Coordinate result;
  hull::diff(result, v1, v2);
  return result;
}

void add(hull::Coordinate &subject, const hull::Coordinate &to_add,
         float to_add_coeff) {
  subject.x += to_add_coeff * to_add.x;
  subject.y += to_add_coeff * to_add.y;
  subject.z += to_add_coeff * to_add.z;
}

class DistanceToSegment {
public:
  DistanceToSegment(const hull::Coordinate &a, const hull::Coordinate &b)
      : a(a) {
    ba_delta = delta(b, a);
    ba_delta_dot = hull::dot(ba_delta, ba_delta);
  }

  float operator()(const hull::Coordinate &point) const {
    float s = hull::dot(delta(point, a), ba_delta) / ba_delta_dot;
    hull::Coordinate result = delta(a, point);
    add(result, ba_delta, s);
    return hull::dot(result, result);
  };

private:
  hull::Coordinate a;
  hull::Coordinate ba_delta;
  float ba_delta_dot;
};

class DistanceToPlane {
public:
  DistanceToPlane(const hull::Coordinate &a, const hull::Coordinate &b,
                  const hull::Coordinate &c)
      : a(a) {
    ba_delta = delta(b, a);
    ba_delta_dot = hull::dot(ba_delta, ba_delta);
    ca_delta = delta(c, a);
    ca_delta_dot = hull::dot(ca_delta, ca_delta);
    mixed_dot = hull::dot(ca_delta, ba_delta);
  }

  float operator()(const hull::Coordinate &point) const {
    auto pa_delta = delta(point, a);
    float c1 = hull::dot(pa_delta, ba_delta);
    float c2 = hull::dot(pa_delta, ca_delta);

    float beta = (mixed_dot * c2 - ca_delta_dot * c1) /
                 (mixed_dot * mixed_dot - ba_delta_dot * ca_delta_dot);
    float gamma = (c1 - beta * ba_delta_dot) / mixed_dot;

    hull::Coordinate result = delta(a, point);
    add(result, ba_delta, beta);
    add(result, ca_delta, gamma);
    return hull::dot(result, result);
  }

private:
  hull::Coordinate a;
  hull::Coordinate ba_delta;
  float ba_delta_dot;
  hull::Coordinate ca_delta;
  float ca_delta_dot;
  float mixed_dot;
};
} // namespace

std::array<std::size_t, 4> PointCloud::getInitialTethraedron() const {
  std::array<std::size_t, 4> result;
  result[0] = 0;

  result[1] = farthest_to_subject(points, [&subject = points[result.front()]](
                                              const hull::Coordinate &point) {
    return hull::squaredDistance(subject, point);
  });

  {
    DistanceToSegment segment_operator(points[result[0]], points[result[1]]);
    result[2] = farthest_to_subject(
        points, [&segment_operator](const hull::Coordinate &point) {
          return segment_operator(point);
        });
  }

  {
    DistanceToPlane plane_operator(points[result[0]], points[result[1]],
                                   points[result[2]]);
    result[3] = farthest_to_subject(
        points, [&plane_operator](const hull::Coordinate &point) {
          return plane_operator(point);
        });
  }

  return result;
}
} // namespace qh
