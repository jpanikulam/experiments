cmake_minimum_required(VERSION 2.8)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}")
include_directories("${BASEPATH}/../cuda/common/inc")

set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

# Specify binary name and source file to build it from
add_executable(
    stviewer
    st_view.cc
)

target_link_libraries(stviewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})