cmake_minimum_required(VERSION 2.8.3)
project(useEigen)

set(cmake_build_type "Release")
set(check_CXX_FLAG "-std = C++11")

# set(EIGEM_INCLUDE_DIR "usr/include/eigen3")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS}$)

include_directories(cmake)
add_executable(useEigen useeigen.cpp)