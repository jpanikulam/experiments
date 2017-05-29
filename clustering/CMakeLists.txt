cmake_minimum_required(VERSION 2.8)

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

#########################
# OpenGL/GLFW
#
find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
find_package(GLEW REQUIRED)

#########################
# Life is what you cmake of it
#

add_executable(meanshift_view meanshift_view.cc)
target_link_libraries(meanshift_view window_2d window_manager ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})
# set_target_properties(meanshift_view PROPERTIES ${CMAKE_CXX_FLAGS} "-O2")
