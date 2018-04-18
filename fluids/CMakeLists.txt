
add_library(fields_2d fields_2d.cc)
add_library(navier_stokes_2d navier_stokes_2d.cc)

target_link_libraries(navier_stokes_2d fields_2d)

add_test(navier_stokes_2d_test navier_stokes_2d_test.cc navier_stokes_2d)

#
# Basic 2d flow
#
add_executable(basic_2d basic_2d.cc)
target_link_libraries(
    basic_2d

    navier_stokes_2d

    window_3d
    window_manager
    plot
    image
    frame
    image_frame
    simple_geometry
)

