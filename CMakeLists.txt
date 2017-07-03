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

# I mean, let's be honest...
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


set(CMAKE_CXX_COMPILER "g++-5")
set(CMAKE_CXX_FLAGS "--std=c++14 -g -O1 -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")

add_subdirectory(clustering)
add_subdirectory(frames)
add_subdirectory(numerics)
add_subdirectory(raytracing)
add_subdirectory(geometry)
add_subdirectory(util)
add_subdirectory(viewer)
add_subdirectory(vision)

