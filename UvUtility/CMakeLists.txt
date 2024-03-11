cmake_minimum_required(VERSION 3.15)
project(UvUtility)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(matplotlib_cpp REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR} 
)
find_package(Python3 COMPONENTS Development NumPy)

file(GLOB UvUtilitySource *.cpp)

add_library(UvUtility ${UvUtilitySource})

target_include_directories(UvUtility PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(UvUtility Python3::Python Python3::NumPy)
