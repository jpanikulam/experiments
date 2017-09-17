cmake_minimum_required(VERSION 2.8)

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

add_subdirectory(primitives)

add_library(simple_window simple_window.cc)
target_link_libraries(simple_window ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})

add_library(window_manager window_manager.cc)
target_link_libraries(window_manager simple_window ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})

add_library(projection projection.cc)
target_link_libraries(projection ${OPENGL_LIBRARIES})

add_library(window_2d window_2d.cc)
target_link_libraries(window_2d projection window_manager spatial_geometry ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})

add_library(window_3d window_3d.cc)
target_link_libraries(
    window_3d

    projection
    window_manager

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
)

add_executable(view_demo view_demo.cc)
target_link_libraries(view_demo
    window_manager
    window_2d

    plot

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
)

add_executable(view_demo_3d view_demo_3d.cc)
target_link_libraries(
    view_demo_3d

    raycaster
    read_stl
    # sphere_volume

    window_manager
    window_3d

    box
    image
    simple_geometry
    plot

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
)
