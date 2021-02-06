/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef QHULL_DISTANCE_MAPPER_POOL_H
#define QHULL_DISTANCE_MAPPER_POOL_H

#include "DistanceMapper.h"
#include <mutex>

namespace qh {
#ifdef THREAD_POOL_ENABLED
    class QuickHullSolver::DistanceMapperPool : public QuickHullSolver::DistanceMapper {
	public:
		DistanceMapperPool(CloudHandler& hndlr, std::shared_ptr<thpl::equi::Pool> pool);

		void AddedChangedFacets(const std::list<const hull::Facet*>& added,const std::list<const hull::Facet*>& changed) const override;

    private:
		std::mutex distanceMapMtx;
		std::shared_ptr<thpl::equi::Pool> pool;
 	};
#endif
}

#endif