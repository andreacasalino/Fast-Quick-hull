/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>
#include <QuickHull/Error.h>
#include <array>
#include <vector>
#ifdef THREAD_POOL_ENABLED
#include <EquiPool.h>
#include <memory>
#endif

namespace qh {
constexpr float QHULL_GEOMETRIC_TOLLERANCE = static_cast<float>(1e-3);

class QuickHullSolver {
public:
#ifdef THREAD_POOL_ENABLED
  /** @brief The size of the thread pool used internally by the solver
   * to compute the future convex hulls.
   * @param when passing 0, no pool is created and the standard serial
   * version of the algorithm will be exevuted when computing new
   * new convex hulls.
   */
#endif
  explicit QuickHullSolver(
#ifdef THREAD_POOL_ENABLED
      const std::size_t &poolSize = 0
#endif
  );

  /** @brief Sets the maximum number of iterations used when
   * building the convex hull in convexHull(...)
   */
  inline void setMaxIterations(const std::size_t &maxIterations) {
    this->maxIterations = maxIterations;
  };

  typedef std::array<std::size_t, 3> FacetIncidences;

  /** @brief The convex hull is built starting from a point cloud described by
   Cloud.
   * Cloud should be an iterable container of V, which is a generic type used to
   describe a
   * 3d point.
   * The typename V must have at least 3 getters returning each coordinate,
   defined in this way (with the obvious meaning of notation):
                  -> const float& V::x() const;
                  -> const float& V::y() const;
                  -> const float& V::z() const;
   * @return the incidences of the facets composing the convex hull: each
   element is a triplet with the positions of the vertices in the cloud
   composing that facet
   */
  template <typename V, typename Cloud>
  std::vector<FacetIncidences> convexHull(const Cloud &cloud) {
    CloudHandlerConcrete<V, Cloud> hndl(cloud);
    std::vector<FacetIncidences> incidences;
    this->_convexHull(incidences, nullptr, hndl);
    return incidences;
  };

  /** @brief The convex hull is built starting from a point cloud described by
   Cloud.
   * Cloud should be an iterable container of V, which is a generic type used to
   describe a
   * 3d point.
   * The typename V must have at least 3 getters returning each coordinate,
   defined in this way (with the obvious meaning of notation):
                  -> const float& V::x() const;
                  -> const float& V::y() const;
                  -> const float& V::z() const;
   * @return a pair of values:
                  -> first: the incidences of the facets composing the convex
   hull: each element is a triplet with the positions of the vertices in the
   cloud composing that facet
                  -> second: the outgoing normals of each facets.
   */
  template <typename V, typename Cloud>
  std::pair<std::vector<FacetIncidences>, std::vector<hull::Coordinate>>
  convexHullWithNormals(const Cloud &cloud) {
    CloudHandlerConcrete<V, Cloud> hndl(cloud);
    std::vector<FacetIncidences> incidences;
    std::vector<hull::Coordinate> normals;
    this->_convexHull(incidences, &normals, hndl);
    return std::make_pair(std::move(incidences), std::move(normals));
  };

private:
  class CloudHandler {
  public:
    virtual std::pair<std::array<hull::Coordinate, 4>,
                      std::array<std::size_t, 4>>
    getInitialTethraedron() = 0;
    virtual std::pair<int, float> getFarthest(const hull::Coordinate &A,
                                              const hull::Coordinate &N) = 0;
    virtual hull::Coordinate getCoordinate(const int &incidence) = 0;
    virtual void invalidate(const int &incidence) = 0;
  };

  template <typename V, typename Cloud>
  class CloudHandlerConcrete : public CloudHandler {
  public:
    CloudHandlerConcrete(const Cloud &cloud) : cloud(tocCloud(cloud)) {
      if (this->cloud.size() < 4) {
        throw Error("a convex 3d cloud should contain at least 4 points");
      }
      this->pointValidity.resize(this->cloud.size(), true);
    };

  private:
    inline static float squaredDistance(const hull::Coordinate &a, const V &b) {
      float res = (a.x - b.x()) * (a.x - b.x());
      res += (a.y - b.y()) * (a.y - b.y());
      res += (a.z - b.z()) * (a.z - b.z());
      return res;
    }

    inline static float squaredDistanceLine(const hull::Coordinate &a,
                                            const hull::Coordinate &b,
                                            const V &c) {
      float V1[3];
      V1[0] = a.x - c.x();
      V1[1] = a.y - c.y();
      V1[2] = a.z - c.z();
      float V2[3];
      V2[0] = b.x - a.x;
      V2[1] = b.y - a.y;
      V2[2] = b.z - a.z;
      float s_opt = -(V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]) /
                    (V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);

      float result =
          2.f * s_opt * (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
      result += s_opt * s_opt * (V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);
      result += (V1[0] * V1[0] + V1[1] * V1[1] + V1[2] * V1[2]);
      return result;
    }

    std::pair<std::array<hull::Coordinate, 4>, std::array<std::size_t, 4>>
    getInitialTethraedron() final {
      std::array<hull::Coordinate, 4> tethra;
      std::array<std::size_t, 4> tethraIndex;
      // 1: farthest from origin
      std::size_t kMax = 0, k;
      float distMax = squaredDistance(hull::ORIGIN, *this->cloud[0]), dist;
      for (k = 1; k < this->cloud.size(); ++k) {
        dist = squaredDistance(hull::ORIGIN, *this->cloud[k]);
        if (dist > distMax) {
          kMax = k;
          distMax = dist;
        }
      }
      tethra[0] = this->getCoordinate(static_cast<int>(kMax));
      tethraIndex[0] = kMax;
      // 2: farthest from first point
      kMax = 0, k;
      distMax = squaredDistance(tethra[0], *this->cloud[0]);
      for (k = 1; k < this->cloud.size(); ++k) {
        dist = squaredDistance(tethra[0], *this->cloud[k]);
        if (dist > distMax) {
          kMax = k;
          distMax = dist;
        }
      }
      if (distMax < QHULL_GEOMETRIC_TOLLERANCE) {
        throw Error("found 0 volume cloud");
      }
      tethra[1] = this->getCoordinate(static_cast<int>(kMax));
      tethraIndex[1] = kMax;
      // 3: farthest from line of first and second
      kMax = 0, k;
      distMax = squaredDistanceLine(tethra[0], tethra[1], *this->cloud[0]);
      for (k = 1; k < this->cloud.size(); ++k) {
        dist = squaredDistanceLine(tethra[0], tethra[1], *this->cloud[k]);
        if (dist > distMax) {
          kMax = k;
          distMax = dist;
        }
      }
      if (distMax < QHULL_GEOMETRIC_TOLLERANCE) {
        throw Error("found 0 volume cloud");
      }
      tethra[2] = this->getCoordinate(static_cast<int>(kMax));
      tethraIndex[2] = kMax;
      // 4: farthest from facet of first, second and third
      hull::Coordinate N = cross(
          hull::Coordinate{tethra[1].x - tethra[0].x, tethra[1].y - tethra[0].y,
                           tethra[1].z - tethra[0].z},
          hull::Coordinate{tethra[2].x - tethra[0].x, tethra[2].y - tethra[0].y,
                           tethra[2].z - tethra[0].z});
      auto temp = this->getFarthest(tethra[0], N);
      if (0.f == temp.first) {
        invert(N);
        temp = this->getFarthest(tethra[0], N);
        if (QHULL_GEOMETRIC_TOLLERANCE == temp.first) {
          throw Error("found 0 volume cloud");
        }
      }
      tethra[3] = this->getCoordinate(temp.first);
      tethraIndex[3] = static_cast<std::size_t>(temp.first);

      for (std::size_t k = 0; k < 4; ++k) {
        this->invalidate(static_cast<int>(tethraIndex[k]));
      }
      return std::make_pair(tethra, tethraIndex);
    }

    static std::vector<const V *> tocCloud(const Cloud &cloud) {
      std::vector<const V *> cl;
      cl.reserve(cloud.size());
      for (auto it = cloud.begin(); it != cloud.end(); ++it) {
        cl.push_back(&(*it));
      }
      return cl;
    };

    std::pair<int, float> getFarthest(const hull::Coordinate &A,
                                      const hull::Coordinate &N) final {
      std::pair<int, float> result =
          std::make_pair(-1, QHULL_GEOMETRIC_TOLLERANCE);
      float dist;
      hull::Coordinate Delta;
      for (std::size_t k = 0; k < this->cloud.size(); ++k) {
        if (this->pointValidity[k]) {
          Delta.x = this->cloud[k]->x() - A.x;
          Delta.y = this->cloud[k]->y() - A.y;
          Delta.z = this->cloud[k]->z() - A.z;
          dist = dot(N, Delta);
          if (dist > result.second) {
            result.second = dist;
            result.first = static_cast<int>(k);
          }
        }
      }
      return result;
    };

    hull::Coordinate getCoordinate(const int &incidence) final {
      const V *pV = this->cloud[incidence];
      return hull::Coordinate{pV->x(), pV->y(), pV->z()};
    };

    inline void invalidate(const int &incidence) {
      this->pointValidity[incidence] = false;
    }

    const std::vector<const V *> cloud;
    std::vector<bool> pointValidity;
  };

  class DistanceMapper;
#ifdef THREAD_POOL_ENABLED
  class DistanceMapperPool;
#endif
  void _convexHull(std::vector<FacetIncidences> &indices,
                   std::vector<hull::Coordinate> *normals, CloudHandler &hndlr);

  // data
  std::size_t maxIterations = 1000;
#ifdef THREAD_POOL_ENABLED
  std::shared_ptr<thpl::equi::Pool> pool;
#endif
};
} // namespace qh
