/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __VECTOR3D_BASIC_H__
#define __VECTOR3D_BASIC_H__

#include <list>

//this is just an example of typename to use for representing 3d coordiante. Since FastQHull is a template class, you can feel free to define and use the one you prefer
class V {
public:
	V(const float& X, const float& Y, const float& Z) { this->_x = X; this->_y = Y; this->_z = Z; };

	const float& x() const { return this->_x; };
	const float& y() const { return this->_y; };
	const float& z()  const { return this->_z; };
private:
	// data
	float		_x;
	float		_y;
	float		_z;
};

/** \brief Used for sampling within the uniform interval [low, upper]
*/
float rand_unif(const float& low, const float& upper) {

	if (upper < low) abort();

	return  low + (upper - low) *  (float)rand() / (float)RAND_MAX;

}

/** \brief Used for sampling N_points 3D coordinates, within a 
cube going from (-1,-1,-1) and (+1,+1,+1)
*/
void sample_cloud(std::list<V>* Cloud, const size_t& N_points) {

	Cloud->clear();
	for (size_t k = 0; k < N_points; k++) {
		Cloud->push_back(V(rand_unif(-1.f, 1.f), rand_unif(-1.f, 1.f), rand_unif(-1.f, 1.f)));
	}

}

#endif 
