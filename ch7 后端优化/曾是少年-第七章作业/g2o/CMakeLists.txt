cmake_minimum_required(VERSION 2.8.3)
project(G2O)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

include_directories(
        ${PROJECT_SOURCE_DIR} 
        ${EIGEN3_INCLUDE_DIR} 
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

add_library(bal_common common.cpp)

add_executable(G2O g2o.cpp )
target_link_libraries(G2O ${G2O_LIBS} ${Sophus_LIBRARIES} bal_common)

