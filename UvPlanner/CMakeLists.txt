cmake_minimum_required(VERSION 3.15)
project(UvPlanner)
set(CMAKE_CXX_STANDARD 17)
include_directories( ./UvUtility ./PathFinder/SampleBased/RRT)
file(GLOB PFSOURCEFILE ./PathFinder/SearchBased/*.cpp  ./PathFinder/SampleBased/RRT/rrt.cc)


find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

add_library(PathFinder STATIC ${PFSOURCEFILE})
add_executable(test ./PathFinder/SampleBased/RRT/test.cc)

target_include_directories(test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(test Python3::Python Python3::NumPy)
target_link_libraries(test PathFinder UvUtility)


target_link_libraries(test PathFinder Python3::Python Python3::NumPy)