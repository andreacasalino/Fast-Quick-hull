All files are contained in ./content/

The functionalities for the convex hull computation are all contained in ./content/src/FastQHull.h
The code is cross-platform: only c++ standard libraries were used.

./content/Sample_01/Main_01.cpp reports an example considering a sampled point cloud

./content/Sample_02/Main_02.cpp reports several examples considering some clouds imported from exemplificative .stl files

Results can be displayed using the script python ./Result_visualization/Main.py

When compiling FastQHull with OpenMP enabled, the multithreaded version is automatically activated.

If you are interested in knowing the theory behind the code refer to./content/Fast_QHull.pdf