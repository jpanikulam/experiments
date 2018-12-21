cmake_minimum_required(VERSION 2.8)

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}/third_party/Sophus")
include_directories("${BASEPATH}/third_party/eigen3")
include_directories("${BASEPATH}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


#########################
# GTest
#
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

#########################
# OpenCV
#
find_package(OpenCV REQUIRED)

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
# ASSIMP
#
find_package(ASSIMP REQUIRED)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/run)

macro(add_test test_name test_file test_libs)
    add_executable(
        ${test_name}
        ${test_file}
    )
    set_target_properties(
        ${test_name}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test
    )
    target_link_libraries(${test_name} ${test_libs} ${GTEST_LIBRARIES} pthread)
endmacro(add_test)

# TODO: Make this run whenever make is run
execute_process(
    COMMAND python ${CMAKE_SOURCE_DIR}/cppmv/cppmv.py -v success -p ${CMAKE_SOURCE_DIR}
)

set(CMAKE_CXX_COMPILER "g++-7")
set(CMAKE_CXX_FLAGS "--std=c++17 -g -O1 -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")


# add_subdirectory(viewer)
add_subdirectory(tmp)
add_subdirectory(vision)
