/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef QHULL_QUICK_HULL_H
#define QHULL_QUICK_HULL_H

#include <list>
#include <array>
#include <vector>
#include <Coordinate.h>
#include <atomic>
#include <Error.h>
#ifdef THREAD_POOL_ENABLED
#include <EquiPool.h>
#include <memory>
#endif

namespace qh {
	constexpr float QHULL_GEOMETRIC_TOLLERANCE = static_cast<float>(1e-3);

	class QuickHullSolver  {
	public:
		QuickHullSolver(
#ifdef THREAD_POOL_ENABLED
		const std::size_t& poolSize = 0
#endif
		);

		inline void setMaxIterations(const std::size_t& maxIterations) { this->maxIterations = maxIterations; };
		
		typedef std::array<std::size_t, 3> FacetIncidences;

		/** @brief The convex hull is built starting from a point cloud described by Cloud.
		 * Cloud should be an iterable container of V, which is a generic type used to describe a
		 * 3d point. 
		 * The typename V must have at least 3 getters returning each coordinate, defined in this way (with the obvious meaning of notation):
				-> const float& V::x() const;
				-> const float& V::y() const;
				-> const float& V::z() const;
		*/
		template<typename V, typename Cloud>
		std::vector<FacetIncidences> convexHull(const Cloud& cloud){
			CloudHandlerConcrete<V, Cloud> hndl(cloud);
			std::vector<FacetIncidences> incidences;
			this->_convexHull(incidences, nullptr, hndl);
			return incidences;
		};

		/** @brief The convex hull is built starting from a point cloud described by Cloud.
		 * Cloud should be an iterable container of V, which is a generic type used to describe a
		 * 3d point. 
		 * The typename V must have at least 3 getters returning each coordinate, defined in this way (with the obvious meaning of notation):
				-> const float& V::x() const;
				-> const float& V::y() const;
				-> const float& V::z() const;
		*/
		template<typename V, typename Cloud>
		std::pair<std::vector<FacetIncidences>
		, std::vector<Coordinate>> convexHullWithNormals(const Cloud& cloud){
			CloudHandlerConcrete<V, Cloud> hndl(cloud);
			std::vector<FacetIncidences> incidences;
			std::vector<Coordinate> normals;
			this->_convexHull(incidences, &normals, hndl);
			return std::make_pair(std::move(incidences), std::move(normals));
		};

	private:
		class CloudHandler {
		public:
			virtual std::array<Coordinate, 4> getInitialTethraedron() = 0;
			virtual std::pair<float, int> getFarthest(const Coordinate& A, const Coordinate& N) = 0;
			virtual Coordinate getCoordinate(const int& incidence) = 0;
			virtual void invalidate(const int& incidence) = 0;
		};

		template<typename V, typename Cloud>
		class CloudHandlerConcrete {
		public:
			CloudHandlerConcrete(const Cloud& cloud) 
				: cloud(tocCloud(cloud)) {
				if(this->cloud.size() < 4) {
					throw Error("a convex 3d cloud should contain at least 4 points");
				}
				this->pointValidity.resize(this->cloud.size(), true);
			};

		private:
			inline static float squaredDistance(const Coordinate& a, const V& b) {
				float res = (a.x - b.x()) * (a.x - b.x());
				res += (a.y - b.y()) * (a.y - b.y());
				res += (a.z - b.z()) * (a.z - b.z());
				return res;
			}

			inline static float squaredDistanceLine(const Coordinate& a, const Coordinate& b, const V& c) {
				float V1[3];
				V1[0] = a.x - c.x();
				V1[1] = a.y - c.y();
				V1[2] = a.z - c.z();
				float V2[3];
				V2[0] = b.x - a.x;
				V2[1] = b.y - a.y;
				V2[2] = b.z - a.z;
				float s_opt = -(V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]) / (V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);

				float result = 2.f * s_opt *(V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
				result += s_opt * s_opt*(V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);
				result += (V1[0] * V1[0] + V1[1] * V1[1] + V1[2] * V1[2]);
				return result;
			}

			std::array<Coordinate, 4> getInitialTethraedron() final {
				std::array<Coordinate, 4> tethra;
			// 1: farthest from origin
				std::size_t kMax =0, k;
				float distMax = squaredDistance(ORIGIN , *this->cloud[0]), dist;
				for(k=1; k<this->cloud.size(); ++k) {
					dist = squaredDistance(ORIGIN, *this->cloud[k]);
					if(dist > distMax){
						kMax = k;
						distMax = dist;
					}
				}
				tethra[0] = this->getCoordinate(kMax);
				this->invalidate(kMax);
			// 2: farthest from first point
				kMax =0, k;
				distMax = squaredDistance(tethra[0] , *this->cloud[0]);
				for(k=1; k<this->cloud.size(); ++k) {
					dist = squaredDistance(tethra[0], *this->cloud[k]);
					if(dist > distMax){
						kMax = k;
						distMax = dist;
					}
				}
				if(distMax < QHULL_GEOMETRIC_TOLLERANCE) {
					throw Error("found 0 volume cloud");
				}
				tethra[1] = this->getCoordinate(kMax);
				this->invalidate(kMax);
			// 3: farthest from line of first and second
				kMax =0, k;
				distMax = squaredDistanceLine(tethra[0], tethra[1], *this->cloud[0]);
				for(k=1; k<this->cloud.size(); ++k) {
					dist = squaredDistanceLine(tethra[0], tethra[1], *this->cloud[k]);
					if(dist > distMax){
						kMax = k;
						distMax = dist;
					}
				}
				if(distMax < QHULL_GEOMETRIC_TOLLERANCE) {
					throw Error("found 0 volume cloud");
				}
				tethra[2] = this->getCoordinate(kMax);
				this->invalidate(kMax);
			// 4: farthest from facet of first, second and third
				Coordinate N = cross(
					Coordinate{tethra[1].x - tethra[0].x, tethra[1].y - tethra[0].y, tethra[1].z - tethra[0].z} ,
					Coordinate{tethra[2].x - tethra[0].x, tethra[2].y - tethra[0].y, tethra[2].z - tethra[0].z} );
				auto temp = this->getFarthest(tethra[0], N);
				if(0.f == temp.first) {
					invert(N);
					temp = this->getFarthest(tethra[0], N);
					if(0.f == temp.first) {
						throw Error("found 0 volume cloud");
					}
				}
				tethra[3] = this->getCoordinate(temp.second);
				this->invalidate(temp.second);

				return tethra;
			}

			static std::vector<const V*> tocCloud(const Cloud& cloud) {
				std::vector<const V*> cl;
				cl.reserve(cloud.size());
				for(auto it= cloud.begin(); it!=cloud.end(); ++it) {
					cl.push_back(&(*it));
				}
				return cl;
			}; 

			std::pair<float, int> getFarthest(const Coordinate& A, const Coordinate& N) final{
				std::pair<float, int> result = std::make_pair(0.f , -1);
				float dist;
				Coordinate Delta;
				for(std::size_t k=0; k<this->cloud.size(); ++k) {
					if(this->pointValidity[k]) {
						Delta.x = it->x() - A.x;
						Delta.y = it->y() - A.y;
						Delta.z = it->z() - A.z;
						dist = dot(N, Delta);
						if(dist > result.second) {
							result.second = dist;
							result.first = k;
						}
					}
				}
				return result;
			};

			Coordinate getCoordinate(const int& incidence) final {
				const V* pV = this->cloud[incidence];
				return Coordinate{pv->x(), pv->y(), pv->z()};
			};

			inline void invalidate(const int& incidence) {
				this->pointValidity[incidence] = false;				
			}

			const std::vector<const V*> cloud;
			std::vector<bool> pointValidity;
		};

		class DistanceMapper;
#ifdef THREAD_POOL_ENABLED
		class DistanceMapperPool;
#endif
		void _convexHull(std::vector<FacetIncidences>& indices, std::vector<Coordinate>* normals, CloudHandler& hndlr);

	// data
		std::atomic<std::size_t> maxIterations = 1000;
#ifdef THREAD_POOL_ENABLED
		std::shared_ptr<thpl::equi::Pool> pool;
#endif
	};
}

#endif 
