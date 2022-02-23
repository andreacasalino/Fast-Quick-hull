/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Hull/Hull.h>
#include <QuickHull/FastQuickHull.h>

#include "DistanceMapper.h"

namespace qh {
namespace {
hull::Hull convex_hull_(PointCloud &points, const ConvexHullContext &cntx) {
  DistanceMapper mapper(points);

  auto initial_tethraedron = points.getInitialTethraedron();
  hull::Hull hull(points.accessVertex(initial_tethraedron[0]),
                  points.accessVertex(initial_tethraedron[1]),
                  points.accessVertex(initial_tethraedron[2]),
                  points.accessVertex(initial_tethraedron[3]));
  hull.setObserver(mapper);
  mapper.update();
  for (const auto index : initial_tethraedron) {
    points.invalidateVertex(index);
  }

  for (std::size_t iteration = 0; (iteration < cntx.max_iterations) &&
                                  (!mapper.getDistancesFacetsMap().empty());
       ++iteration) {
    const auto &[facet, vertex] =
        mapper.getDistancesFacetsMap().begin()->second;
    hull.update(points.accessVertex(vertex), facet);
    mapper.update();
    points.invalidateVertex(vertex);
  }

  return hull;
}

std::vector<FacetIncidences>
get_indices(const std::vector<hull::Facet> &faces) {
  std::vector<FacetIncidences> result;
  result.reserve(faces.size());
  for (const auto &face : faces) {
    result.push_back(FacetIncidences{face.vertexA, face.vertexB, face.vertexC});
  }
  return result;
}

std::vector<hull::Coordinate>
get_normals(const std::vector<hull::Facet> &faces) {
  std::vector<hull::Coordinate> result;
  result.reserve(faces.size());
  for (const auto &face : faces) {
    result.push_back(face.normal);
  }
  return result;
}
} // namespace

std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            const ConvexHullContext &cntx) {
  PointCloud cloud(points);
  auto hull = convex_hull_(cloud, cntx);
  return get_indices(hull.getFacets());
}

std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            const ConvexHullContext &cntx,
            std::vector<hull::Coordinate> &convex_hull_normals) {
  PointCloud cloud(points);
  auto hull = convex_hull_(cloud, cntx);
  convex_hull_normals = get_normals(hull.getFacets());
  return get_indices(hull.getFacets());
}

} // namespace qh
