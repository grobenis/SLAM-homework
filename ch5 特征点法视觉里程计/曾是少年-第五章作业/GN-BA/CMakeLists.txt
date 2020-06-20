cmake_minimum_required(VERSION 2.8.3)
project(GNBA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)

include_directories(${Sophus_INCLUDE_DIRS})

add_executable(GNBA GN-BA.cpp)
target_link_libraries(GNBA ${Sophus_LIBRARIES})