cmake_minimum_required(VERSION 2.8.3)
project(DirectBA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(Pangolin REQUIRED)

# SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(
        ${EIGEN3_INCLUDE_DIR} 
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        ${PANGOLIN_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
)

add_executable(DirectBA directBA.cpp )
target_link_libraries(DirectBA ${G2O_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

