#include "PointCloud.h"
#include <QuickHull/Error.h>

#include "Definitions.h"

namespace qh {
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

PointCloud::FarthestVertex
PointCloud::getFarthest(const hull::Coordinate &point_on_facet,
                        const hull::Coordinate &facet_normal) const {
  PointCloud::FarthestVertex result = PointCloud::FarthestVertex{0, 0.f};
  float distance;
  for (std::size_t pos = 0; pos < points.size(); ++pos) {
    if (points_still_free[pos]) {
      distance = get_vertex_distance(point_on_facet, facet_normal, points[pos]);
      if (result.distance < distance) {
        result.distance = distance;
        result.vertex = pos;
      }
    }
  }
  return result;
}

namespace {
template <typename DistanceComputation>
std::size_t
farthest_to_subject(const std::vector<hull::Coordinate> &points,
                    const DistanceComputation &distance_to_subject) {
  float max_distance = QHULL_GEOMETRIC_TOLLERANCE_SQUARED;
  std::size_t result = 0;
  float squared_distance;
  for (std::size_t k = 0; k < points.size(); ++k) {
    squared_distance = distance_to_subject(points[k]);
    if (squared_distance > max_distance) {
      result = k;
      max_distance = squared_distance;
    }
  }
  if (max_distance <= QHULL_GEOMETRIC_TOLLERANCE_SQUARED) {
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
         const float to_add_coeff) {
  subject.x += to_add_coeff * to_add.x;
  subject.y += to_add_coeff * to_add.y;
  subject.z += to_add_coeff * to_add.z;
}

class DistanceToSegment {
public:
  DistanceToSegment(const hull::Coordinate &a, const hull::Coordinate &b)
      : a(a), b_minus_a(delta(b, a)),
        b_minus_a_dot(hull::dot(b_minus_a, b_minus_a)) {}

  float operator()(const hull::Coordinate &point) const {
    float s = hull::dot(delta(point, a), b_minus_a) / b_minus_a_dot;
    hull::Coordinate result = delta(a, point);
    add(result, b_minus_a, s);
    return hull::dot(result, result);
  };

private:
  const hull::Coordinate a;
  const hull::Coordinate b_minus_a;
  const float b_minus_a_dot;
};

class DistanceToPlane {
public:
  DistanceToPlane(const hull::Coordinate &a, const hull::Coordinate &b,
                  const hull::Coordinate &c)
      : a(a), b_minus_a(delta(b, a)),
        b_minus_a_dot(hull::dot(b_minus_a, b_minus_a)), c_minus_a(delta(c, a)),
        c_minus_a_dot(hull::dot(c_minus_a, c_minus_a)),
        b_minus_a_dot_c_minus_a(hull::dot(b_minus_a, c_minus_a)) {}

  float operator()(const hull::Coordinate &point) const {
    float c1 = hull::dot(delta(point, a), b_minus_a);
    float c2 = hull::dot(delta(point, a), c_minus_a);

    float beta = (c_minus_a_dot * c1 - b_minus_a_dot_c_minus_a * c2) /
                 (b_minus_a_dot * c_minus_a_dot -
                  b_minus_a_dot_c_minus_a * b_minus_a_dot_c_minus_a);
    float gamma = (c2 - beta * b_minus_a_dot_c_minus_a) / c_minus_a_dot;

    hull::Coordinate result = delta(a, point);
    add(result, b_minus_a, beta);
    add(result, c_minus_a, gamma);
    return hull::dot(result, result);
  }

private:
  const hull::Coordinate a;
  const hull::Coordinate b_minus_a;
  const float b_minus_a_dot;
  const hull::Coordinate c_minus_a;
  const float c_minus_a_dot;
  const float b_minus_a_dot_c_minus_a;
};
} // namespace

std::vector<std::size_t> PointCloud::getInitialTethraedron() const {
  std::vector<std::size_t> result;
  result.reserve(4);

  result.push_back(0);

  result.push_back(farthest_to_subject(
      points,
      [&subject = points[result.front()]](const hull::Coordinate &point) {
        return hull::squaredDistance(subject, point);
      }));

  DistanceToSegment segment_operator(points[result[0]], points[result[1]]);
  result.push_back(farthest_to_subject(
      points, [&segment_operator](const hull::Coordinate &point) {
        return segment_operator(point);
      }));

  DistanceToPlane plane_operator(points[result[0]], points[result[1]],
                                 points[result[2]]);
  result.push_back(farthest_to_subject(
      points, [&plane_operator](const hull::Coordinate &point) {
        return plane_operator(point);
      }));

  return result;
}
} // namespace qh
