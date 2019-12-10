/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../src/Log_utils.h"
#include "../src/STL.h"
using namespace std;


int main() {

	//import the cloud from existing STL files: uncomment the one for which you want to compute the point cloud (pay attention to the relative path to use)
	list<V> imported_cloud;
	//Import_Vertices(&imported_cloud , "./Animals_stl/Dolphin.stl"); //672 vertices
	//Import_Vertices(&imported_cloud, "./Animals_stl/Giraffe.stl"); //788 vertices
	Import_Vertices(&imported_cloud, "./Animals_stl/Hyppo.stl"); //2399 vertices
	//Import_Vertices(&imported_cloud, "./Animals_stl/Snake.stl"); //3691 vertices
	//Import_Vertices(&imported_cloud, "./Animals_stl/Eagle.stl"); //4271 vertices

	//compute the convex hull and put the info in a log file (check the initial commands inside Log_as_JSON_CovexHull
	// for understanding how to use Fast_QHull)
	Log_as_JSON_CovexHull( imported_cloud , "../Result_visualization/Log");

	system("pause");
	return 0;
}