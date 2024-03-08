cmake_minimum_required(VERSION 3.15)
project(UvKits)
set(CMAKE_CXX_STANDARD 17)
set(UVCOMPONENTS
    UvMap
    UvPlanner
    UvTaskAllocator
    UvUtility
)
subdirs(${UVCOMPONENTS})