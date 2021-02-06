/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "src/ImporterSTL.h"
#include <iostream>
using namespace std;


int main() {
	// create a solver
	qh::QuickHullSolver solver
#ifdef THREAD_POOL_ENABLED // try to use 4 threads for the pool
		(4)
#endif
		;

	//import the cloud from existing STL files
	list<std::string> stlNames;
	stlNames.push_back("Dolphin"); //672 vertices
	stlNames.push_back("Giraffe"); //788 vertices
	stlNames.push_back("Hyppo"); //2399 vertices
	stlNames.push_back("Snake"); //3691 vertices
	stlNames.push_back("Eagle"); //4271 vertices

	for (auto it =stlNames.begin(); it!=stlNames.end(); ++it) {
		// impor the stl and compute the convex hull. Then, log the results
		// You can use the python script Plotter.py to display the results
		cout << "computing convex hull of " << *it;
		logConvexhull(solver, importSTL("Animals/" + *it + ".stl"), *it + ".json");
		cout << " done" << endl;
	}

	return EXIT_SUCCESS;
}