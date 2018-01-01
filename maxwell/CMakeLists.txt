add_library(
    physics
    physics.cc
)

add_executable(
    mag_view
    mag_view.cc
)

target_link_libraries(mag_view
    physics

    geometry
    optimization
    window_2d
    window_manager
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
)
