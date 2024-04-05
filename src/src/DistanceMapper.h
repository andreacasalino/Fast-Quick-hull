/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Hull.h>
#include <QuickHull/FastQuickHull.h>

#include "PointCloud.h"

#include <atomic>
#include <optional>
#include <set>
#include <unordered_map>

namespace qh {
class DistanceMapper : public hull::Observer {
public:
  DistanceMapper(const PointCloud &cloud) : cloud(cloud){};

  void processLastUpdate();

  struct FacetVertexDistance {
    const hull::Facet *facet;
    std::size_t vertex_index;
    float distance;

    bool operator<(const FacetVertexDistance &o) const {
      return distance < o.distance;
    }
  };

  using Distances = std::multiset<FacetVertexDistance>;

  const FacetVertexDistance *getBest() const {
    return distances.empty() ? nullptr : &(*distances.rbegin());
  }

  void hullChanges(const hull::Observer::Notification &notification) override {
    last_notification.emplace(notification);
  };

protected:
  const PointCloud &cloud;

  std::optional<hull::Observer::Notification> last_notification;

  std::atomic_bool spin_lock = true;
  std::unordered_map<const hull::Facet *, Distances::iterator> facets_table;
  Distances distances;

  std::optional<FacetVertexDistance> recompute(const hull::Facet *facet) const;
};
} // namespace qh
