/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __LOG_UTILS_H__
#define __LOG_UTILS_H__

#include <fstream>
#include <string>
#include "../src/Vector3d_basic.h"
#include "FastQHull.h"

/** \brief Accepts a point clound and internally computes the convex hull.
Then, the info describing the convex hull are saved in the file whose location
is described by log_file. You can use Result_visualization/Render.html to visualize the results.
*/
void Log_as_JSON_CovexHull(const std::list<V>& cloud, const std::string& log_file, const size_t& Iterations = 0) {

	//build a new solver
	Fast_QHull<V> convexHull_solver;

	//compute the convex hull of the cloud
	convexHull_solver.Compute_new_Convex_Hull(&cloud, Iterations);

	//take from the solver the incidences of the facets
	size_t N_facets;
	size_t* incidences = convexHull_solver.Get_Incidences(&N_facets);

	//take the normals of the facets
	std::list<V> normals;
	convexHull_solver.Get_Normals(&normals);


	//put the above info in a JSON file for displaying it, using Result_visualization/Render.html

	std::ofstream f(log_file);
	if (!f.is_open()) abort();

	f << "{\n";

	f << "\"Cloud\":[\n";
	auto point_end = cloud.end();
	point_end--;
	for (auto it = cloud.begin(); it != cloud.end(); it++) {
		f << "[" << it->x() << "," << it->y() << "," <<it->z()  << "]";
		if (it != point_end)
			f << ",";
		f << std::endl;
	}
	f << "],\n";

	f << "\"Index\":[\n";
	for (size_t k = 0; k < N_facets; k++) {
		f << "[" << incidences[3 * k] << "," << incidences[3 * k + 1] << "," << incidences[3 * k + 2] << "]";
		if (k < (N_facets - 1))
			f << ",";
		f << std::endl;
	}
	f << "],\n";

	f << "\"Normals\":[\n";
	auto norm_end = normals.end();
	norm_end--;
	for (auto it = normals.begin(); it != normals.end(); it++) {
		f << "[" << it->x() << "," << it->y() << "," << it->z() << "]";
		if (it != norm_end)
			f << ",";
		f << std::endl;
	}
	f << "]\n";

	f << "}";

	f.close();

}

#endif