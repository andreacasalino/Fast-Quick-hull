/**
 * Author:    Andrea Casalino
 * Created:   11.05.2009
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../src/Log_utils.h"
using namespace std;


int main() {

	//randomly sample a point cloud of a given number of points
	size_t Numb_point = 50;
	list<V> cloud;
	sample_cloud(&cloud, Numb_point);

	//compute the convex hull and put the info in a log file (check the initial commands inside Log_as_JSON_CovexHull
	// for understanding how to use Fast_QHull)
	Log_as_JSON_CovexHull(cloud, "../Result_visualization/Log");


	system("pause");
	return 0;
}