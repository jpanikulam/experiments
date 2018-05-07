cmake_minimum_required(VERSION 2.8)

#########################
# Life is what you cmake of it
#

add_library(geometry geometry.cc)
add_test(geometry_test geometry_test.cc geometry)

add_library(optimization optimization.cc)
add_test(optimization_test optimization_test.cc optimization)

add_library(ransac ransac.cc)
target_link_libraries(ransac window_2d)
add_test(ransac_test ransac_test.cc ransac)

add_executable(view view.cc)
target_link_libraries(view geometry optimization window_2d window_manager ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES} ${GLEW_LIBRARIES})
