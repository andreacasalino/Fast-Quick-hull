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
#include <unordered_map>

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

using HullIndexVSPointCloudIndexMap =
    std::unordered_map<std::size_t, std::size_t>;

hull::Hull convex_hull_(PointCloud &points, const ConvexHullContext &cntx,
                        HullIndexVSPointCloudIndexMap &indices_map) {
  DistanceMapper mapper(points);
  indices_map.clear();

  auto initial_tethraedron = points.getInitialTethraedron();
  hull::Hull hull(points.points[initial_tethraedron[0]],
                  points.points[initial_tethraedron[1]],
                  points.points[initial_tethraedron[2]],
                  points.points[initial_tethraedron[3]], mapper);

  indices_map.emplace(0, initial_tethraedron[0]);
  indices_map.emplace(1, initial_tethraedron[1]);
  indices_map.emplace(2, initial_tethraedron[2]);
  indices_map.emplace(3, initial_tethraedron[3]);

  std::atomic_bool life = true;
  auto pool_size = get_pool_size(cntx.thread_pool_size);
#pragma omp parallel num_threads(pool_size)
  {
    auto th_id = omp_get_thread_num();
    if (0 == th_id) {
      for (const auto index : initial_tethraedron) {
        points.closeVertex(index);
      }
#pragma omp barrier
      mapper.processLastUpdate();
#pragma omp barrier

      for (std::size_t iteration = 0; iteration <= cntx.max_iterations;
           ++iteration) {
        const auto *furthest = mapper.getBest();
        if (furthest == nullptr) {
          break;
        }
        indices_map.emplace(hull.getContext().vertices.size(),
                            furthest->vertex_index);
        hull.update(points.points[furthest->vertex_index],
                    const_cast<hull::Facet *>(furthest->facet));
        points.closeVertex(furthest->vertex_index);
#pragma omp barrier
        mapper.processLastUpdate();
#pragma omp barrier
      }
      life.store(false);
#pragma omp barrier
    } else {
      while (true) {
#pragma omp barrier
        if (!life.load()) {
          break;
        }
        mapper.processLastUpdate();
#pragma omp barrier
      }
    }
  }

  return hull;
}

std::vector<FacetIncidences>
get_indices(const hull::HullContext &ctxt,
            const HullIndexVSPointCloudIndexMap &indices_map) {
  std::vector<FacetIncidences> result;
  result.reserve(ctxt.faces.size());
  for (const auto &face : ctxt.faces) {
    result.emplace_back(FacetIncidences{indices_map.at(face->vertexA),
                                        indices_map.at(face->vertexB),
                                        indices_map.at(face->vertexC)});
  }
  return result;
}

std::vector<hull::Coordinate> get_normals(const hull::HullContext &ctxt) {
  std::vector<hull::Coordinate> result;
  result.reserve(ctxt.faces.size());
  for (const auto &face : ctxt.faces) {
    result.emplace_back(face->normal);
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
  return get_indices(hull.getContext(), indices_map);
}

std::vector<FacetIncidences>
convex_hull(const std::vector<hull::Coordinate> &points,
            std::vector<hull::Coordinate> &convex_hull_normals,
            const ConvexHullContext &cntx) {
  PointCloud cloud(points);
  HullIndexVSPointCloudIndexMap indices_map;
  auto hull = convex_hull_(cloud, cntx, indices_map);
  convex_hull_normals = get_normals(hull.getContext());
  return get_indices(hull.getContext(), indices_map);
}

} // namespace qh
