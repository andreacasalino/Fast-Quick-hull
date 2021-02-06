/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapperPool.h"

namespace qh {
#ifdef THREAD_POOL_ENABLED
    QuickHullSolver::DistanceMapperPool::DistanceMapperPool(CloudHandler& hndlr, std::shared_ptr<thpl::equi::Pool> pool)
        : DistanceMapper(hndlr) 
        , pool(pool) {
    }

    void QuickHullSolver::DistanceMapperPool::AddedChangedFacets(const std::list<const hull::Facet*>& added,const std::list<const hull::Facet*>& changed) const {        
        for(auto it = changed.begin(); it!=changed.end(); ++it) {
            this->pool->push([this, it](){
                auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
                std::lock_guard<std::mutex> mapLock(this->distanceMapMtx);
                if(farthest.first > 0) {
                    this->distanceMap.find(*it)->second = farthest;
                }
                else {
                    this->distanceMap.erase(this->distanceMap.find(*it));
                }
            });
        }
        for(auto it = added.begin(); it!=added.end(); ++it) {
            this->pool->push([this, it](){
                auto farthest = this->hndlr.getFarthest(*(*it)->A, (*it)->N);
                if(farthest.first > 0) {
                    std::lock_guard<std::mutex> mapLock(this->distanceMapMtx);
                    this->distanceMap.emplace(*it, farthest);
                }
            });
        }
        this->pool->wait();
    };
#endif
}