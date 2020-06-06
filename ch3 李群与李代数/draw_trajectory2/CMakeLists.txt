cmake_minimum_required(VERSION 2.8.3)
project(drawtraj)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so) #显示定义路径
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
add_executable(drawtraj  rmse.cpp)
target_link_libraries(drawtraj ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})


