cmake_minimum_required(VERSION 3.15)
project(UvPlanner)
set(CMAKE_CXX_STANDARD 17)
include_directories( ./UvUtility ./PathFinder/SampleBased/RRT ./PathFinder/SearchBased)
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

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 
add_executable(search_test ./PathFinder/SearchBased/search_test.cpp)
target_link_libraries(search_test ${OpenCV_LIBS} PathFinder)


include_directories(./PathFinder/Trajectory)
file(GLOB TRAJ_SOURCEFILE ./PathFinder/Trajectory/*.cpp)
add_library(traj_lib STATIC ${TRAJ_SOURCEFILE})
add_executable(b_test ./PathFinder/Trajectory/b_test.cc)
target_include_directories(b_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
target_link_libraries(b_test traj_lib UvUtility Python3::Python Python3::NumPy)