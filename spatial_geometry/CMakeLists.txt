cmake_minimum_required(VERSION 2.8)

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# Life is what you cmake of it
#

set(CMAKE_CXX_COMPILER "g++-5")
set (CMAKE_CXX_FLAGS "--std=c++14 -g -fno-omit-frame-pointer -Wall ${CMAKE_CXX_FLAGS}")

add_library(spatial_geometry geometry.cc plane.cc)
