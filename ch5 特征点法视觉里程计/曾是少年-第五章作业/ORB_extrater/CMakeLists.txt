cmake_minimum_required(VERSION 2.8.3)
project(orb)

set(CMAKE_CXX_STANDARD 17)
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

find_package(OpenCV REQUIRED)

add_executable(orb computeORB.cpp)
target_link_libraries(orb ${OpenCV_LIBS})