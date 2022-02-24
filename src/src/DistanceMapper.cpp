/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapper.h"
#include <omp.h>

namespace qh {

void DistanceMapper::updateAddedFacet(const hull::Facet *facet) {
  auto result = cloud.getFarthest(
      last_notification->context.vertices[facet->vertexA], facet->normal);
  std::scoped_lock map_lock(maps_mtx);
  facets_distances_map[facet] = result.distance;
  distances_facets_map.emplace(result.distance,
                               FacetAndFarthestVertex{facet, result.vertex});
}

void DistanceMapper::updateChangedFacet(const hull::Facet *facet) {
  auto result = cloud.getFarthest(
      last_notification->context.vertices[facet->vertexA], facet->normal);
  updateRemovedFacet(facet);
  std::scoped_lock map_lock(maps_mtx);
  facets_distances_map[facet] = result.distance;
  distances_facets_map.emplace(result.distance,
                               FacetAndFarthestVertex{facet, result.vertex});
}

void DistanceMapper::updateRemovedFacet(const hull::Facet *facet) {
  std::scoped_lock map_lock(maps_mtx);
  auto facets_distances_map_it = facets_distances_map.find(facet);
  auto range =
      distances_facets_map.equal_range(facets_distances_map_it->second);
  for (auto distances_facets_map_it = range.first;
       distances_facets_map_it != range.second; ++distances_facets_map_it) {
    if (distances_facets_map_it->second.facet == facet) {
      distances_facets_map.erase(distances_facets_map_it);
      break;
    }
  }
  facets_distances_map.erase(facets_distances_map_it);
}

void DistanceMapper::update() {
// changed facets
#pragma omp for
  for (const auto *changed : last_notification->changed) {
    updateChangedFacet(changed);
  }
  // added facets
#pragma omp for
  for (const auto *added : last_notification->added) {
    updateAddedFacet(added);
  }
// removed facets
#pragma omp for
  for (const auto &removed : last_notification->removed) {
    updateRemovedFacet(removed.get());
  }
}
} // namespace qh
