/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapper.h"
#include <omp.h>

namespace qh {
namespace {
struct Guard {
  Guard(std::atomic_bool &lock) : lock_{lock} {
    while (true) {
      bool expected = true;
      if (lock_.compare_exchange_strong(
              expected, false, std::memory_order::memory_order_acquire)) {
        break;
      }
    }
  }
  ~Guard() { lock_.store(true, std::memory_order::memory_order_release); }

private:
  std::atomic_bool &lock_;
};
} // namespace

std::optional<DistanceMapper::FacetVertexDistance>
DistanceMapper::recompute(const hull::Facet *facet) const {
  const auto &point = last_notification->context.vertices[facet->vertexA];
  auto info = cloud.getFarthest(point, facet->normal);
  std::optional<FacetVertexDistance> res;
  if (info.has_value()) {
    res.emplace(FacetVertexDistance{facet, info->vertex, info->distance});
  }
  return res;
}

void DistanceMapper::processLastUpdate() {
  // changed facets
#pragma omp for
  for (int i = 0; i < last_notification->changed.size(); ++i) {
    const auto *facet = last_notification->changed[i];
    auto info = recompute(facet);
    Guard guard{spin_lock};
    auto it = facets_table.find(facet);
    distances.erase(it->second);
    if (info.has_value()) {
      it->second = distances.emplace(info.value());
    }
  }
  // added facets
#pragma omp for
  for (int i = 0; i < last_notification->added.size(); ++i) {
    const auto *facet = last_notification->added[i];
    auto info = recompute(facet);
    if (info.has_value()) {
      Guard guard{spin_lock};
      facets_table[facet] = distances.emplace(info.value());
    }
  }
  // removed facets
#pragma omp for
  for (int i = 0; i < last_notification->removed.size(); ++i) {
    const auto *facet = last_notification->removed[i];
    Guard guard{spin_lock};
    auto it = facets_table.find(facet);
    distances.erase(it->second);
    facets_table.erase(it);
  }
}
} // namespace qh
