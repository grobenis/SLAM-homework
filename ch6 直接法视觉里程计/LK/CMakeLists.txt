cmake_minimum_required(VERSION 2.8.3)
project(LK)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
)

add_executable(LK optical_flow.cpp )
target_link_libraries(LK ${OpenCV_LIBS})
