cmake_minimum_required(VERSION 2.8)
find_package(CUDA QUIET REQUIRED)


#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# OpenGL
#
find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS})

#########################
# GLFW
#
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

#########################
# GLEW
#
find_package(GLEW REQUIRED)

#########################
# Life is what you cmake of it
#
set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
include_directories("${BASEPATH}/third_party/Sophus")
include_directories("${BASEPATH}")

set(CMAKE_CXX_COMPILER "g++-5")

set (CMAKE_CXX_FLAGS "--std=c++14 -g -fno-omit-frame-pointer -Wall ${CMAKE_CXX_FLAGS}")


add_library(window_manager window_manager.cc)
add_library(simple_window simple_window.cc)
target_link_libraries(simple_window ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})
target_link_libraries(window_manager simple_window ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})

add_library(window_2d window_2d.cc)
target_link_libraries(window_2d ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})

add_executable(view_demo view_demo.cc)
target_link_libraries(view_demo window_manager window_2d ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})
