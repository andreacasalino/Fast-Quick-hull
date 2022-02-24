/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Hull/Hull.h>
#include <QuickHull/FastQuickHull.h>

#include "DistanceMapper.h"

#include <algorithm>
#include <omp.h>

namespace qh {
namespace {
int get_default_pool_size() {
  int result = 0;
#pragma omp parallel
  {
    if (0 == omp_get_thread_num()) {
      result = omp_get_num_threads();
    }
  }
  return result;
}

static const int DEAFULT_POOL_SIZE = get_default_pool_size();

int get_pool_size(const std::optional<std::size_t> &thread_pool_size) {
  int pool_size = 1;
  if (thread_pool_size != std::nullopt) {
    if (*thread_pool_size <= 1) {
      pool_size = DEAFULT_POOL_SIZE;
    } else {
      pool_size = *thread_pool_size;
    }
  }
  return pool_size;
}

hull::Hull convex_hull_(PointCloud &points, const ConvexHullContext &cntx) {
  DistanceMapper mapper(points);

  auto initial_tethraedron = points.getInitialTethraedron();
  hull::Hull hull(points.accessVertex(initial_tethraedron[0]),
                  points.accessVertex(initial_tethraedron[1]),
                  points.accessVertex(initial_tethraedron[2]),
                  points.accessVertex(initial_tethraedron[3]), mapper);

  bool life = true;
  auto pool_size = get_pool_size(cntx.thread_pool_size);
#pragma omp parallel num_threads(pool_size)
  {
    auto th_id = omp_get_thread_num();
    if (0 == th_id) {
#pragma omp barrier
      mapper.update();
      for (const auto index : initial_tethraedron) {
        points.invalidateVertex(index);
      }

      for (std::size_t iteration = 0; (iteration < cntx.max_iterations) &&
                                      (!mapper.getDistancesFacetsMap().empty());
           ++iteration) {
        const auto &[facet, vertex] =
            mapper.getDistancesFacetsMap().begin()->second;
        auto facets_it =
            std::find_if(hull.getFacets().begin(), hull.getFacets().end(),
                         [&facet = facet](const hull::FacetPtr &element) {
                           return facet == element.get();
                         });
        hull.update(points.accessVertex(vertex),
                    std::distance(hull.getFacets().begin(), facets_it));
#pragma omp barrier
        mapper.update();
        points.invalidateVertex(vertex);
      }
      life = false;
#pragma omp barrier
    } else {
      while (true) {
#pragma omp barrier
        if (!life) {
          break;
        }
        mapper.update();
      }
    }
  }

  return hull;
}

std::vector<FacetIncidences> get_indices(const hull::Facets &faces) {
  std::vector<FacetIncidences> result;
  result.reserve(faces.size());
  for (const auto &face : faces) {
    result.push_back(
        FacetIncidences{face->vertexA, face->vertexB, face->vertexC});
  }
  return result;
}

std::vector<hull::Coordinate> get_normals(const hull::Facets &faces) {
  std::vector<hull::Coordinate> result;
  result.reserve(faces.size());
  for (const auto &face : faces) {
    result.push_back(face->normal);
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
