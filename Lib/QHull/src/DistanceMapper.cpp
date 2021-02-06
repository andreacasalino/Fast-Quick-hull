/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapper.h"

namespace qh {
    void QuickHullSolver::DistanceMapper::AddedChangedFacets(const std::list<const hull::Facet*>& added,const std::list<const hull::Facet*>& changed) const {
        for(auto it = changed.begin(); it!=changed.end(); ++it) {
            auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
            if(farthest.second > QHULL_GEOMETRIC_TOLLERANCE) {
                this->distanceMap.find(*it)->second = farthest;
            }
            else {
                this->distanceMap.erase(this->distanceMap.find(*it));
            }
        }
        for(auto it = added.begin(); it!=added.end(); ++it) {
            auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
            if(farthest.second > QHULL_GEOMETRIC_TOLLERANCE) {
                this->distanceMap.emplace(*it, farthest);
            }
        }
    };

    void QuickHullSolver::DistanceMapper::RemovedFacets(const std::list<const hull::Facet*>& removed) const {
        for(auto it = removed.begin(); it!=removed.end(); ++it) {
            this->distanceMap.erase(this->distanceMap.find(*it));
        }
    };
}