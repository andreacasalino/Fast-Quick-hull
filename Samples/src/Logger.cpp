/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Logger.h"
#include <fstream>

float get(const std::size_t& pos, const Vector3d& v) {
    switch (pos) {
    case 0:
        return v.x();
    case 1:
        return v.y();
    default:
        return v.z();
    }
    return v.z();
}

std::size_t get(const std::size_t& pos, const std::array<std::size_t, 3>& i) {
    return i[pos];
}

float get(const std::size_t& pos, const qh::Coordinate& v) {
    switch (pos) {
    case 0:
        return v.x;
    case 1:
        return v.y;
    default:
        return v.z;
    }
    return v.z;
}

template<typename T>
void log(const T& e, std::ofstream& f) {
    f << '[' << get(0, e) << ',' << get(1, e) << ',' << get(2, e) << ']' << std::endl;
}

template<typename C>
void log(const C& c, const std::string& name, std::ofstream& f) {
    f << '\"' << name << "\":[" << std::endl; 
    if(!c.empty()) {
        auto it =c.begin();
        log(*it, f);
        ++it;
        for(it; it!=c.end(); ++it){
            f << ',';
            log(*it, f);
        }
    }
    f << ']' << std::endl;
}

void logConvexhull(qh::QuickHullSolver& solver, const std::list<Vector3d>& cloud, const std::string& fileName) {
    auto ch = solver.convexHullWithNormals(cloud);

    std::ofstream f(fileName);
    if(!f.is_open()) return;

    f << '{' << std::endl;
    log(cloud, "Cloud", f);
    f << ',';
    log(ch.first, "Index", f);
    f << ',';
    log(ch.second, "Normals", f);
    f << '}';
}
