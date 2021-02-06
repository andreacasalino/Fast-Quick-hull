/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "src/Logger.h"
using namespace std;

void sampleCloud(std::list<Vector3d>& cloud, const std::size_t& size);

int main() {
	//randomly sample a point cloud of a given number of points
	list<Vector3d> cloud;
	sampleCloud(cloud, 50);

	// create a solver
	qh::QuickHullSolver solver;

	// Compute the convex hull and put the info in a log file.
	// You can use the python script Plotter.py to display the result
	logConvexhull(solver, cloud, "Sample01.json");

	return EXIT_SUCCESS;
}

float sample() {
	return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.f;
}

void sampleCloud(std::list<Vector3d>& cloud, const std::size_t& size) {
	cloud.clear();
	for (std::size_t k=0; k<size; ++k) {
		cloud.emplace_back( sample(), sample(), sample());
	}
}