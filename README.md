This library contains the implementations of the **Quick Hull** algorithm, which is able to efficiently computes the **convex hull** of point clouds.
If you are interested in the theoretical aspects behind the algorithm have a look at ./doc/Fast_QHull.pdf.

This library is stand-alone and completely **cross platform**. Use [CMake](https://cmake.org) to configure the project.

You can exploit an internal thread pool strategy to compute the **convex hull** of clouds made of thousands of points. 
From the outside you just need to specify the size of the pool.
However, if you don't need such an advance approach (and not compile redundant code), you can select **OFF** for the 
[CMake](https://cmake.org) option **THREAD_POOL_OPT**. Beware, that by default such an option is set as **ON**.

The relevant code is contained in ./Lib, while ./Samples contains samples showing how to use this library.
In particular, after running the samples, some .json files will be produced storing the results.
You can use the script pyhton ./Samples/Plotter.py to visualize each result in a **3d plot**, by running

 * `python Plotter.py $(name of the .json to plot)`
 
**Compile**

 * (**ONLY FOR USING THE THREAD POOL STRATEGY, OTHERWISE SKIP**) After cloning the library you need to initialize a git submodule by running the following commands from the root folder:
   
   * `git submodule init`
   * `git submodule update`
   
 * Configure and generate the project using [CMake](https://cmake.org)
 
 ![Example of convex hulls](https://github.com/andreacasalino/XML-GUI/blob/master/Example.png)