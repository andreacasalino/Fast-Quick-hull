/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapper.h"
#include <omp.h>

namespace qh {

void DistanceMapper::updateAddedFacet(const std::size_t facet_index) {
  const auto &facet = last_notification->facets[facet_index];
  const auto &facet_vertex = last_notification->vertices[facet.vertexA];
  auto result = cloud.getFarthest(facet_vertex, facet.normal);
  if (nullptr != result) {
    facets_distances_map[facet_index] = result->distance;
    distances_facets_map.emplace(
        result->distance, FacetAndFarthestVertex{facet_index, result->vertex});
  }
}

void DistanceMapper::updateChangedFacet(const std::size_t facet_index) {
  const auto &facet = last_notification->facets[facet_index];
  const auto &facet_vertex = last_notification->vertices[facet.vertexA];
  auto result = cloud.getFarthest(facet_vertex, facet.normal);
  updateRemovedFacet(facet_index);
  if (nullptr != result) {
    facets_distances_map[facet_index] = result->distance;
    distances_facets_map.emplace(
        result->distance, FacetAndFarthestVertex{facet_index, result->vertex});
  }
}

void DistanceMapper::updateRemovedFacet(const std::size_t facet_index) {
  auto facets_distances_map_it = facets_distances_map.find(facet_index);
  auto range =
      distances_facets_map.equal_range(facets_distances_map_it->second);
  for (auto distances_facets_map_it = range.first;
       distances_facets_map_it != range.second; ++distances_facets_map_it) {
    if (distances_facets_map_it->second.facet_index == facet_index) {
      distances_facets_map.erase(distances_facets_map_it);
      break;
    }
  }
  facets_distances_map.erase(facets_distances_map_it);
}

void DistanceMapper::update() {
  throw 0; // use reverse order for the distances map

  // added facets
#pragma omp for
  for (const auto added : last_notification->added) {
    updateAddedFacet(added);
  }
// changed facets
#pragma omp for
  for (const auto changed : last_notification->added) {
    updateChangedFacet(changed);
  }
// removed facets
#pragma omp for
  for (const auto &removed : last_notification->removed) {
    updateRemovedFacet(removed.old_index);
  }
#pragma omp barrier
}
} // namespace qh