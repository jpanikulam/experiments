cmake_minimum_required(VERSION 2.8)

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}/third_party/Sophus")
include_directories("${BASEPATH}")

add_subdirectory(spatial_geometry)
add_subdirectory(viewer)

add_subdirectory(raytracing)