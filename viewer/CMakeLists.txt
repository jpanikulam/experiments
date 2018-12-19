cmake_minimum_required(VERSION 2.8)

#########################
# Life is what you cmake of it
#

add_subdirectory(interaction)
add_subdirectory(primitives)
add_subdirectory(colors)

add_library(simple_window simple_window.cc)
target_link_libraries(simple_window ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})

add_library(window_manager window_manager.cc)
target_link_libraries(window_manager
    simple_window
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
    pthread
)

add_library(projection projection.cc)
target_link_libraries(projection ${OPENGL_LIBRARIES})

add_library(window_2d window_2d.cc)
target_link_libraries(window_2d
    projection
    window_manager

    plane
    ray_ray_approx_intersect

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
)

add_library(gl_renderable_buffer gl_renderable_buffer.cc)
target_link_libraries(gl_renderable_buffer
    projection

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
)

add_library(window_3d window_3d.cc)
target_link_libraries(
    window_3d

    view3d
    projection
    window_manager

    camera

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

find_package(CUDA QUIET)
if (CUDA_FOUND)
    add_subdirectory(cuda)
endif(CUDA_FOUND)
