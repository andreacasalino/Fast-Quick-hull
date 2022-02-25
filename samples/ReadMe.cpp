#include <QuickHull/FastQuickHull.h>

// A 3d coordinate representation
struct MyVector {
  float coordinate_x;
  float coordinate_y;
  float coordinate_z;
};

hull::Coordinate convert_function(const MyVector &vector) {
  return hull::Coordinate{vector.coordinate_x, vector.coordinate_y,
                          vector.coordinate_z};
};

int main() {
  // A collection of 3d coordinates
  std::vector<MyVector> points;

  // compute the incidences (index of the vertices at the boundaries
  // of each facet), of the facets constituting the convex hull of the points
  std::vector<qh::FacetIncidences>
      incidences = // qh::FacetIncidences is an array of incidences:
                   // std::array<std::size_t, 3>
      qh::convex_hull(points.begin(), points.end(), convert_function);

  // compute at the same time the convex hull facets incidences and normals
  std::vector<hull::Coordinate> normals;
  incidences =
      qh::convex_hull(points.begin(), points.end(), convert_function, normals);

  {
    // specify in the context that we want to exploit ALL the available cores of
    // this machine
    qh::ConvexHullContext context;
    context.thread_pool_size = 0;
    std::vector<hull::Coordinate> normals;
    incidences = qh::convex_hull(points.begin(), points.end(), convert_function,
                                 normals, context);
  }

  {
    // specify in the context the thread pool size
    qh::ConvexHullContext context;
    context.thread_pool_size = 3; // 3 threads will be used
    std::vector<hull::Coordinate> normals;
    incidences = qh::convex_hull(points.begin(), points.end(), convert_function,
                                 normals, context);
  }

  return EXIT_SUCCESS;
}
