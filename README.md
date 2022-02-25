This library contains the implementations of the **Quick Hull** algorithm, which is able to efficiently computes the **convex hull** of point clouds.
If you are interested in the theoretical aspects behind the algorithm have a look at the [documentation](./doc/Fast_QHull.pdf).

It is also possible to rely on a **multi threaded** version of the **Quick Hull** algorithm, see the **MULTI THREADING** section.

This library is stand-alone and completely **cross platform**. Use [CMake](https://cmake.org) to configure the project.

![Example of convex hulls](https://github.com/andreacasalino/Fast-Quick-hull/blob/master/CH.png)

## SAMPLES

The relevant code is contained in ./src, while ./samples contains examples showing how to use this library.
In particular, after running the samples, some .json files will be produced storing the results.
You can use the python script Plotter.py to visualize results in cool **3d plot**, by running:

 * `python3 Plotter.py ${JSON_TO_PLOT_NAME}`

## USAGE

Haven't yet left a **star**? Do it now! :).

Using this library is pretty easy. The **Quick Hull** algorithm implementation is included into 1 single file, which can be included in this way:
```cpp
#include <QuickHull/FastQuickHull.h>
```

The points in the cloud can be described by any kind of 3d representation ([Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) Vector3d, or any other representation of your favourite linear algebra library), like for example a struct like this:
```cpp
// A 3d coordinate representation
struct MyVector {
  float coordinate_x;
  float coordinate_y;
  float coordinate_z;
};
```

The trick is just to define a conversion function that map your 3d representation to the one uised by this library:
```cpp
hull::Coordinate convert_function(const MyVector &vector) {
  return hull::Coordinate{vector.coordinate_x, vector.coordinate_y,
                          vector.coordinate_z};
};
```

Now you are free to compute the **convex hull** of a cloud of points:
```cpp
// A collection of 3d coordinates
std::vector<MyVector> points;

// compute the incidences (index of the vertices in the passed points cloud,
// which delimits each facet), of the facets constituting the convex hull of
// the points
std::vector<qh::FacetIncidences>
    incidences = // qh::FacetIncidences is an array of incidences:
                // std::array<std::size_t, 3>
    qh::convex_hull(points.begin(), points.end(), convert_function);
```

It is also possible to get at the same time both the **convex hull** facets as well as their outward normals:
```cpp
// compute at the same time the convex hull facets incidences and normals
std::vector<hull::Coordinate> normals;
incidences =
    qh::convex_hull(points.begin(), points.end(), convert_function, normals);
```

## MULTI THREADING

You can exploit an internal thread pool strategy to compute the **convex hull** of clouds made of thousands of points. 

This can be easily done by specifying a context structure that is passed to the function computing the **convex hull**.

In case you want to exploit all the available threads of your machine you can do this:
```cpp
// specify in the context that we want to exploit ALL the available cores of
// this machine
qh::ConvexHullContext context;
context.thread_pool_size = 0;
std::vector<hull::Coordinate> normals;
incidences = qh::convex_hull(points.begin(), points.end(), convert_function,
                            normals, context);
```

Or you can specify the number of threads to use in this way:
```cpp
// specify in the context the thread pool size
qh::ConvexHullContext context;
context.thread_pool_size = 3; // 3 threads will be used
std::vector<hull::Coordinate> normals;
incidences = qh::convex_hull(points.begin(), points.end(), convert_function,
                                normals, context);
```

## CMAKE SUPPORT

Haven't yet left a **star**? Do it now! :).
   
To consume this library you can rely on [CMake](https://cmake.org).
More precisely, You can fetch this package and link to the **Fast-Quick-Hull** library, which will expose the **convex hulll** algorithm:

```cmake
include(FetchContent)
FetchContent_Declare(
qhull_lib
GIT_REPOSITORY https://github.com/andreacasalino/Fast-Quick-hull.git
GIT_TAG        main
)
FetchContent_MakeAvailable(qhull_lib)
```
and then link to the **Fast-Quick-Hull** library:

```cmake
target_link_libraries(${TARGET_NAME}
    Fast-Quick-Hull
)
```
