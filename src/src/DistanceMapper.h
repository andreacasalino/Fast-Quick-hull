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
#include <mutex>

namespace qh {
struct FacetAndFarthestVertex {
  const hull::Facet *facet;
  std::size_t vertex_index;
};
using DistancesFacetsMap =
    std::multimap<float, FacetAndFarthestVertex, std::greater<float>>;

class DistanceMapper : public hull::Observer {
public:
  DistanceMapper(const PointCloud &cloud) : cloud(cloud){};

  void update();

  const DistancesFacetsMap &getDistancesFacetsMap() const {
    return distances_facets_map;
  };

protected:
  const PointCloud &cloud;

  std::unique_ptr<Notification> last_notification;
  void hullChanges(Notification &&notification) override {
    last_notification = std::make_unique<Notification>(std::move(notification));
  };

  void updateAddedFacet(const hull::Facet *facet);
  void updateChangedFacet(const hull::Facet *facet);
  void updateRemovedFacet(const hull::Facet *facet);

  std::mutex maps_mtx;
  DistancesFacetsMap distances_facets_map;
  std::map<const hull::Facet *, float> facets_distances_map;
};
} // namespace qh
