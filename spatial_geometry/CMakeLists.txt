cmake_minimum_required(VERSION 2.8)

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# Life is what you cmake of it
#

add_library(spatial_geometry geometry.cc plane.cc)
add_test(perp_test perp_test.cc "")
