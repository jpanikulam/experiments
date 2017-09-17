cmake_minimum_required(VERSION 2.8)

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# Life is what you cmake of it
#

add_library(spatial_geometry plane.cc)

add_test(perp_test perp_test.cc "")

add_library(ray ray.cc)

add_library(ray_ray_approx_intersect ray_ray_approx_intersect.cc plane.cc)
add_test(ray_ray_approx_intersect_test ray_ray_approx_intersect_test.cc ray_ray_approx_intersect)


add_subdirectory(import)
add_subdirectory(spatial)
