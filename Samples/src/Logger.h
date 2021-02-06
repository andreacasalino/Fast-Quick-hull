/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef SAMPLE_LOGGER_H
#define SAMPLE_LOGGER_H

/** @brief Just an example of 3d vector: you can use the one defined
 * in your favourite linear algebra library (Eigen, etc...)
 */
class Vector3d {
public:
	Vector3d(const float& x, const float& y, const float& z);

	inline float x() const { return this->coordinates[0]; };
	inline float y() const { return this->coordinates[1]; };
	inline float z() const { return this->coordinates[2]; };

private:
	float coordinates[3];
};

#include <string>
#include <FastQuickHull.h>

void logConvexhull(qh::QuickHullSolver& solver, const std::list<Vector3d>& cloud, const std::string& fileName);

#endif
