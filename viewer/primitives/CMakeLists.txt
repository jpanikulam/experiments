cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )

add_library(box box.cc)
target_link_libraries(
    box
    ${OPENGL_LIBRARIES}
)

add_library(image image.cc)

target_link_libraries(
    image

    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
)


add_library(simple_geometry_primitives simple_geometry_primitives.cc)
target_link_libraries(
    simple_geometry_primitives

    ${OPENGL_LIBRARIES}
)

add_library(simple_geometry simple_geometry.cc)
target_link_libraries(
    simple_geometry

    simple_geometry_primitives

    ${OPENGL_LIBRARIES}
)

add_library(plot plot.cc)
target_link_libraries(
    plot

    ${OPENGL_LIBRARIES}
)

add_library(frame frame.cc)
target_link_libraries(
    frame
    ${OPENGL_LIBRARIES}
)

add_library(image_frame image_frame.cc)
target_link_libraries(
    image_frame

    frame
    ${OPENGL_LIBRARIES}
)
