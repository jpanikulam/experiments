cmake_minimum_required(VERSION 2.8)

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}/third_party/Sophus")
include_directories("${BASEPATH}")

#########################
# GTest
#
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

function(add_test test_name test_file test_libs)
    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name} ${test_libs} ${GTEST_LIBRARIES} pthread)
endfunction(add_test)

add_subdirectory(clustering)
add_subdirectory(numerics)
add_subdirectory(raytracing)
add_subdirectory(spatial_geometry)
add_subdirectory(viewer)