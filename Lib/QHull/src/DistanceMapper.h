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

		void AddedChangedFacets(const std::list<const hull::Facet*>& added,const std::list<const hull::Facet*>& changed) const override {
			for(auto it = changed.begin(); it!=changed.end(); ++it) {
				auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
				if(farthest.first > 0) {
					this->distanceMap.find(*it)->second = farthest;
				}
				else {
					this->distanceMap.erase(this->distanceMap.find(*it));
				}
			}
			for(auto it = added.begin(); it!=added.end(); ++it) {
				auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
				if(farthest.first > 0) {
					this->distanceMap.emplace(*it, farthest);
				}
			}
		};

		void RemovedFacets(const std::list<const hull::Facet*>& removed) const override {
			for(auto it = removed.begin(); it!=removed.end(); ++it) {
				this->distanceMap.erase(this->distanceMap.find(*it));
			}
		};

		inline const std::map<const hull::Facet*, std::pair<float, int>>& getDistanceMap() const { return this->distanceMap; };

	protected:
		CloudHandler& hndlr;
		mutable std::map<const hull::Facet*, std::pair<int, float>> distanceMap;
 	};
}

#endif