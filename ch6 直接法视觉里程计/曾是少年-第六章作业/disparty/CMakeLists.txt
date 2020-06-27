cmake_minimum_required(VERSION 2.8.3)
project(DISP)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(
        ${SOPHUS_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(DISP disp.cpp )
target_link_libraries(DISP ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})
