cmake_minimum_required(VERSION 3.15)
project(UvUtility)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
message(STATUS "eigen = ${EIGEN3_INCLUDE_DIR}")
include_directories(
    ${EIGEN3_INCLUDE_DIR} 
)