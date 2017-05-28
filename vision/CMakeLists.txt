if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "DEBUG")
  # SET(CMAKE_BUILD_TYPE "RELEASE")
  # SET(CMAKE_BUILD_TYPE "RELWITHDEBINFO")
  # SET(CMAKE_BUILD_TYPE "MINSIZEREL")
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# So we can use those cool looking angle brackets
set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${BASEPATH}")
# I mean, let's be honest...
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_COMPILER "g++-5")
set(CMAKE_CXX_FLAGS "--std=c++14 -g -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")

find_package( OpenCV REQUIRED )

add_library(features features.cc)
target_link_libraries(features ${OpenCV_LIBS})

add_library(visualization visualization.cc)
target_link_libraries(visualization ${OpenCV_LIBS})

add_library(io io.cc)
target_link_libraries(io ${OpenCV_LIBS})

add_library(images images.cc)
target_link_libraries(images)

add_executable(main main.cc)
target_link_libraries(main features images io ${OpenCV_LIBS})
