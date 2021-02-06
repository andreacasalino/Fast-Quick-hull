/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef SAMPLE_IMPORTER_STL_H
#define SAMPLE_IMPORTER_STL_H

#include "Logger.h"

/** @brief Import an .STL file (https://en.wikipedia.org/wiki/STL_(file_format) as a point cloud of vertices.
*/
std::list<Vector3d> importSTL(const std::string& stlFileName);

#endif  
