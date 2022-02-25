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
#include <map>
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
    if (*thread_pool_size == 0) {
      pool_size = DEAFULT_POOL_SIZE;
    } else {
      pool_size = static_cast<int>(*thread_pool_size);
    }
  }
  return pool_size;
}

using HullIndexVSPointCloudIndexMap = std::map<std::size_t, std::size_t>;

hull::Hull convex_hull_(PointCloud &points, const ConvexHullContext &cntx,
                        HullIndexVSPointCloudIndexMap &indices_map) {
  DistanceMapper mapper(points);
  indices_map.clear();

  auto initial_tethraedron = points.getInitialTethraedron();
  hull::Hull hull(points.accessVertex(initial_tethraedron[0]),
                  points.accessVertex(initial_tethraedron[1]),
                  points.accessVertex(initial_tethraedron[2]),
                  points.accessVertex(initial_tethraedron[3]), mapper);

  indices_map.emplace(0, initial_tethraedron[0]);
  indices_map.emplace(1, initial_tethraedron[1]);
  indices_map.emplace(2, initial_tethraedron[2]);
  indices_map.emplace(3, initial_tethraedron[3]);

  bool life = true;
  auto pool_size = get_pool_size(cntx.thread_pool_size);
#pragma omp parallel num_threads(pool_size)
  {
    auto th_id = omp_get_thread_num();
    if (0 == th_id) {
      for (const auto index : initial_tethraedron) {
        points.invalidateVertex(index);
      }
#pragma omp barrier
      mapper.update();
#pragma omp barrier

      for (std::size_t iteration = 0; (iteration <= cntx.max_iterations) &&
                                      (!mapper.getDistancesFacetsMap().empty());
           ++iteration) {
        const auto &[facet, vertex] =
            mapper.getDistancesFacetsMap().begin()->second;
        auto facets_it =
            std::find_if(hull.getFacets().begin(), hull.getFacets().end(),
                         [&facet = facet](const hull::FacetPtr &element) {
                           return facet == element.get();
                         });
        indices_map.emplace(hull.getVertices().size(), vertex);
        hull.update(points.accessVertex(vertex),
                    std::distance(hull.getFacets().begin(), facets_it));
        points.invalidateVertex(vertex);
#pragma omp barrier
        mapper.update();
#pragma omp barrier
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
#pragma omp barrier
      };
    }
  }

  return hull;
}

std::vector<FacetIncidences>
get_indices(const hull::Facets &faces,
            const HullIndexVSPointCloudIndexMap &indices_map) {
  std::vector<FacetIncidences> result;
  result.reserve(faces.size());
  for (const auto &face : faces) {
    result.push_back(FacetIncidences{indices_map.find(face->vertexA)->second,
                                     indices_map.find(face->vertexB)->second,
                                     indices_map.find(face->vertexC)->second});
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
  HullIndexVSPointCloudIndexMap indices_map;
  auto hull = convex_hull_(cloud, cntx, indices_map);
  return get_indices(hull.getFacets(), indices_map);
}

std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            std::vector<hull::Coordinate> &convex_hull_normals,
            const ConvexHullContext &cntx) {
  PointCloud cloud(points);
  HullIndexVSPointCloudIndexMap indices_map;
  auto hull = convex_hull_(cloud, cntx, indices_map);
  convex_hull_normals = get_normals(hull.getFacets());
  return get_indices(hull.getFacets(), indices_map);
}

} // namespace qh
