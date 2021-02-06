/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef QHULL_DISTANCE_MAPPER_H
#define QHULL_DISTANCE_MAPPER_H

#include <FastQuickHull.h>
#include "Hull.h"
#include <map>

namespace qh {
    class QuickHullSolver::DistanceMapper : public hull::Observer {
	public:
		DistanceMapper(CloudHandler& hndlr) : hndlr(hndlr) {};

		void AddedChangedFacets(const std::list<const hull::Facet*>& added,const std::list<const hull::Facet*>& changed) const override;

		void RemovedFacets(const std::list<const hull::Facet*>& removed) const override;

		inline std::map<const hull::Facet*, std::pair<int, float>>& getDistanceMap() const { return this->distanceMap; };

	protected:
		CloudHandler& hndlr;
		mutable std::map<const hull::Facet*, std::pair<int, float>> distanceMap;
 	};
}

#endif