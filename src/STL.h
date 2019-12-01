/**
 * Author:    Andrea Casalino
 * Created:   11.05.2009
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef  __STL_IMPORT_H__
#define __STL_IMPORT_H__

#include <list>
#include <fstream>
#include <string>
#include <sstream>
#include "Vector3d_basic.h"

#define TOLLERANCE_CLONE (float)1e-4

void splitta_riga(std::string& riga, std::list<std::string>* slices) {

	std::istringstream iss(riga);
	slices->clear();
	while (true) {
		if (iss.eof()) {
			break;
		}
		slices->push_back(std::string());
		iss >> slices->back();
	}

}

/** \brief Import an .STL file (https://en.wikipedia.org/wiki/STL_(file_format) as a point cloud o vertices.
*/
void Import_Vertices(std::list<V>* imported_vertices , const std::string& file_location) {

	imported_vertices->clear();

	std::ifstream f(file_location);
	if (!f.is_open())
		abort(); //invalid file

	float distance;
	std::string line;
	std::list<std::string> slices;

	size_t k;
	std::getline(f, line);
	float V_temp[3];
	bool is_new;
	auto it_V = imported_vertices->begin();
	float toll2 = TOLLERANCE_CLONE * TOLLERANCE_CLONE;
	while (true) {
		std::getline(f, line);
		splitta_riga(line, &slices);
		if (slices.front().compare("endsolid") == 0)
			break;

		std::getline(f, line);
		for (k = 0; k < 3; k++) {
			std::getline(f, line);
			splitta_riga(line, &slices);
			slices.pop_front();
			V_temp[0] = (float)atof(slices.front().c_str()); slices.pop_front();
			V_temp[1] = (float)atof(slices.front().c_str()); slices.pop_front(); 
			V_temp[2] = (float)atof(slices.front().c_str());

			is_new = true;
			for (it_V = imported_vertices->begin(); it_V != imported_vertices->end(); it_V++) {
				distance = (V_temp[0] - it_V->x())*(V_temp[0] - it_V->x());
				distance += (V_temp[1] - it_V->y())*(V_temp[1] - it_V->y());
				distance += (V_temp[2] - it_V->z())*(V_temp[2] - it_V->z());
				if (distance<toll2) {
					is_new = false;
					break;
				}
			}
			if (is_new) 
				imported_vertices->push_back(V(V_temp[0], V_temp[1], V_temp[2] ));
		}
		std::getline(f, line);
		std::getline(f, line);
	}
	f.close();

}

#endif  
