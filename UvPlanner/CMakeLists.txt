cmake_minimum_required(VERSION 3.15)
project(UvPlanner)
set(CMAKE_CXX_STANDARD 17)
include_directories( ./UvUtility ./PathFinder/SampleBased/RRT)
file(GLOB PFSOURCEFILE ./PathFinder/SearchBased/*.cpp  ./PathFinder/SampleBased/RRT/rrt.cc)

find_package(PythonLibs REQUIRED)
#下面两句代码路径要改成你自己的
set(PYTHON_INCLUDE_DIRS /home/ballade/anaconda3/envs/pytorch-gpu/include/python3.8 /home/ballade/anaconda3/envs/pytorch-gpu/lib/python3.8/site-packages/numpy/core/include)
set(PYTHON_LIBRARIES /home/ballade/anaconda3/envs/pytorch-gpu/lib/libpython3.8.so)
include_directories(${PYTHON_INCLUDE_DIRS})
 
# 添加 Matplotlib 的头文件路径  路径改成你自己的
include_directories(/home/kangkang/matplotlib-cpp/)


message(STATUS "source = ${PFSOURCEFILE}")
add_library(PathFinder STATIC ${PFSOURCEFILE})
add_executable(test ./PathFinder/SampleBased/RRT/test.cc)
target_link_libraries(test PathFinder ${PYTHON_LIBRARIES})