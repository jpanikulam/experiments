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
include_directories(${OpenCV_INCLUDE_DIRS})

#########################
# OpenGL
#
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

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
# OpenCL
#
find_package(OpenCL REQUIRED)
include_directories(${OpenCL_INCLUDE_DIRS})

#########################
# ASSIMP
#
find_package(ASSIMP REQUIRED)

#########################
# Freetype
#
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

#########################
# YAML-Cpp
# TODO: Destroy this
#
find_package(yaml-cpp REQUIRED)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/run)
SET(GENERATED_PATH ${PROJECT_BINARY_DIR}/include)

execute_process(
    COMMAND pymake -c "${CMAKE_CURRENT_SOURCE_DIR}/pymake.json" -v success -p "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE ret
)
if(ret EQUAL "1")
    message(FATAL_ERROR "Could not run pymake; run sudo pip install generate-cmake")
endif()

set(CMAKE_CXX_COMPILER "g++-7")
set(CMAKE_CXX_FLAGS "--std=c++17 -g -O1 -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")


find_package(rosbag REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  rosbag
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(cmake_tools)
add_subdirectory(third_party)
add_subdirectory(tmp)

target_compile_definitions(environment PRIVATE ASSET_PATH="${BASEPATH}/data")
target_compile_definitions(environment PRIVATE REPO_PATH="${BASEPATH}")

add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)
add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)
add_definitions(-DBACKWARD_HAS_BFD=1)
