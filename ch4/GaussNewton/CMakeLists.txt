cmake_minimum_required(VERSION 2.8.3)
project(GN)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(GN gaussnewton.cpp)
target_link_libraries(GN ${OpenCV_LIBS} ${Pangolin_LIBRARIES})


