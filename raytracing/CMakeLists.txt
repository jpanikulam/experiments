cmake_minimum_required(VERSION 2.8)
find_package(CUDA QUIET REQUIRED)


#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# OpenGL/GLUT
#
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

#########################
# GTest
#
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


#########################
# Life is what you cmake of it
#
set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
include_directories("${BASEPATH}/third_party/Sophus")
include_directories("${BASEPATH}")
# include_directories("${BASEPATH}/common/inc")

set (CMAKE_CXX_FLAGS "--std=c++11 -g -fno-omit-frame-pointer -Wall ${CMAKE_CXX_FLAGS}")
# list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-O1;-DVERBOSE;-gencode;arch=compute_61,code=sm_61")
# list(APPEND CUDA_NVCC_FLAGS "-std=c++11;-fno-omit-framepointer;-DVERBOSE;-gencode;arch=compute_61,code=sm_61")
# SET(CUDA_PROPAGATE_HOST_FLAGS OFF)


function(add_test test_name test_file test_libs)
    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name} ${test_libs} ${GTEST_LIBRARIES} pthread)
endfunction(add_test)


add_library(geometry geometry.cc)
add_library(optimization optimization.cc)
# add_library(numdiff optimization.cc)

add_test(geometry_test geometry_test.cc geometry)
add_test(optimization_test optimization_test.cc optimization)
add_test(numdiff_test numdiff_test.cc "")


# Specify binary name and source file to build it from
# cuda_add_executable(
#     sim
#     view.cc
#     sim.cu
# )

# target_link_libraries(sim ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
