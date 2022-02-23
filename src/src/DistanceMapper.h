/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "PointCloud.h"
#include <Hull/Hull.h>
#include <QuickHull/FastQuickHull.h>

#include <map>
#include <memory>

namespace qh {
class DistanceMapper : public hull::Observer {
public:
  DistanceMapper(PointCloud &cloud) : cloud(cloud){};

  void update();
  // void update(const std::size_t calling_thread_id);

  struct FarthestVertexAndDistance {
    std::size_t vertex_index;
    float distance;
  };
  const std::map<const hull::Facet *, FarthestVertexAndDistance> &
  getFacetsDistancesMap() const;

  struct FacetAndFarthestVertex {
    std::size_t vertex_index;
    std::size_t facet_index;
  };
  const std::multimap<float, FacetAndFarthestVertex> &
  getDistancesFacetsMap() const;

protected:
  PointCloud &cloud;

  std::unique_ptr<Notification> last_notification;
  void hullChanges(const Notification &notification) override;

  std::map<const hull::Facet *, FarthestVertexAndDistance> facets_distances_map;
  std::multimap<float, FacetAndFarthestVertex> distances_facets_map;
};
} // namespace qh
