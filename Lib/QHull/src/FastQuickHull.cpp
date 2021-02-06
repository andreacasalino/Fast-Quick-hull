/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DistanceMapper.h"
#include "DistanceMapperPool.h"
#include <memory>

namespace qh {
	QuickHullSolver::QuickHullSolver(
#ifdef THREAD_POOL_ENABLED
		const std::size_t& poolSize
#endif
	) {
#ifdef THREAD_POOL_ENABLED
		if(1 < poolSize) {
			this->pool = std::make_shared<thpl::equi::Pool>(poolSize);
		}
#endif		
	};

	void QuickHullSolver::_convexHull(std::vector<FacetIncidences>& indices, std::vector<Coordinate>* normals, CloudHandler& hndlr) {
		std::unique_ptr<DistanceMapper> obs;
#ifdef THREAD_POOL_ENABLED
		if(nullptr == this->pool){
			obs = std::make_unique<DistanceMapper>(hndlr);
		}
		else {
			obs = std::make_unique<DistanceMapperPool>(hndlr, this->pool);
		}
#else
		obs = std::make_unique<DistanceMapper>(hndlr);
#endif

		std::array<Coordinate, 4> tethraedron = hndlr.getInitialTethraedron();
		hull::Hull hull(tethraedron[0], tethraedron[1], tethraedron[2], tethraedron[3], obs.get());

		std::map<const Coordinate*, std::size_t> indiceMap;
		const std::map<const hull::Facet*, std::pair<int, float>>* distMap = &obs->getDistanceMap();
		std::map<const hull::Facet*, std::pair<int, float>>::const_iterator farthest, itF;
		for(std::size_t iter = 0; iter<this->maxIterations; ++iter) {
			if(distMap->empty()) break;
			farthest = distMap->begin();
			for(itF = farthest++; itF!=distMap->end(); ++itF) {
				if(itF->second.second > farthest->second.second) {
					farthest = itF;
				}
			}
			hull.UpdateHull(hndlr.getCoordinate(farthest->second.second),  *farthest->first);
			indiceMap.emplace(&hull.getVertices().back(), static_cast<std::size_t>(farthest->second.second));
			hndlr.invalidate(farthest->second.first);
		}

		// get incidences
		indices.reserve(hull.getFacets().size());
		for(auto f= hull.getFacets().begin(); f!=hull.getFacets().end(); ++f) {
			indices.emplace_back();
			indices.back()[0] = indiceMap.find(f->A)->second;
			indices.back()[1] = indiceMap.find(f->B)->second;
			indices.back()[2] = indiceMap.find(f->C)->second;
		}

		if(nullptr != normals) {
			// get normals
			normals->reserve(hull.getFacets().size());
			for(auto f= hull.getFacets().begin(); f!=hull.getFacets().end(); ++f) {
				normals->emplace_back(f->N);
			}
		}
	}
}

