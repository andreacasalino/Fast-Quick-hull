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

#include <map>
#include <memory>

namespace qh {
class DistanceMapper : public hull::Observer {
public:
  DistanceMapper(const PointCloud &cloud) : cloud(cloud){};

  void update();

  struct FacetAndFarthestVertex {
    std::size_t facet_index;
    std::size_t vertex_index;
  };
  const std::multimap<float, FacetAndFarthestVertex> &
  getDistancesFacetsMap() const {
    return distances_facets_map;
  };

protected:
  const PointCloud &cloud;

  std::unique_ptr<Notification> last_notification;
  void hullChanges(const Notification &notification) override {
    last_notification = std::make_unique<Notification>(notification);
  };

  void updateAddedFacet(const std::size_t facet_index);
  void updateChangedFacet(const std::size_t facet_index);
  void updateRemovedFacet(const std::size_t facet_index);

  std::multimap<float, FacetAndFarthestVertex> distances_facets_map;
  std::map<std::size_t, float> facets_distances_map;
};
} // namespace qh
