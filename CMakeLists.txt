cmake_minimum_required(VERSION 3.15)
project(UvKits)
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(UVCOMPONENTS
    UvMap
    UvPlanner
    UvTaskAllocator
    UvUtility
)
subdirs(${UVCOMPONENTS})

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy)

include_directories(UvUtility UvTaskAllocator UvMap UvPlanner/PathFinder)

# add_executable(testVisual testVisual.cpp)
# target_include_directories(testVisual PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
# target_link_libraries(testVisual Python3::Python Python3::NumPy)
# target_link_libraries(testVisual PathFinder UvUtility)