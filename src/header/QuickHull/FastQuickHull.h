/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

namespace qh {
using FacetIncidences = std::array<std::size_t, 3>;

struct ConvexHullContext {
  std::size_t max_iterations = 1000;
  std::optional<std::size_t> thread_pool_size = std::nullopt;
};

/** @brief The convex hull is built starting from a point cloud described by
 Cloud.
 * Cloud should be an iterable container of V, which is a generic type used to
 describe a
 * 3d point.
 * The typename V must have at least 3 getters returning each coordinate,
 defined in this way (with the obvious meaning of notation):
                -> const float& V::x() const;
                -> const float& V::y() const;
                -> const float& V::z() const;
 * @return the incidences of the facets composing the convex hull: each
 element is a triplet with the positions of the vertices in the cloud
 composing that facet
 */
std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            const ConvexHullContext &cntx = ConvexHullContext{});

template <typename VerticesIterator, typename CoordinateConverter>
std::vector<FacetIncidences>
convex_hull(const VerticesIterator &vertices_begin,
            const VerticesIterator &vertices_end,
            const CoordinateConverter &converter,
            const ConvexHullContext &cntx = ConvexHullContext{}) {
  std::vector<hull::Coordinate> points;
  points.reserve(std::distance(vertices_begin, vertices_end));
  std::for_each(vertices_begin, vertices_end,
                [&points, &converter](const auto &element) {
                  points.push_back(converter(element));
                });
  return convex_hull(points, cntx);
};

/** @brief The convex hull is built starting from a point cloud described by
 Cloud.
 * Cloud should be an iterable container of V, which is a generic type used to
 describe a
 * 3d point.
 * The typename V must have at least 3 getters returning each coordinate,
 defined in this way (with the obvious meaning of notation):
                -> const float& V::x() const;
                -> const float& V::y() const;
                -> const float& V::z() const;
 * @return a pair of values:
                -> first: the incidences of the facets composing the convex
 hull: each element is a triplet with the positions of the vertices in the
 cloud composing that facet
                -> second: the outgoing normals of each facets.
 */
std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            std::vector<hull::Coordinate> &convex_hull_normals,
            const ConvexHullContext &cntx = ConvexHullContext{});

template <typename VerticesIterator, typename CoordinateConverter>
std::vector<FacetIncidences>
convex_hull(const VerticesIterator &vertices_begin,
            const VerticesIterator &vertices_end,
            const CoordinateConverter &converter,
            std::vector<hull::Coordinate> &convex_hull_normals,
            const ConvexHullContext &cntx = ConvexHullContext{}) {
  std::vector<hull::Coordinate> points;
  std::for_each(vertices_begin, vertices_end,
                [&points, &converter](const auto &element) {
                  points.push_back(converter(element));
                });
  return convex_hull(points, convex_hull_normals, cntx);
};

} // namespace qh
