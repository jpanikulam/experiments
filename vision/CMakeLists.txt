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

find_package( OpenCV REQUIRED )

add_library(features features.cc)
target_link_libraries(features ${OpenCV_LIBS})
add_test(features_test features_test.cc features ${OpenCV_LIBS})


add_library(visualization visualization.cc)
target_link_libraries(visualization ${OpenCV_LIBS})

add_library(io io.cc)
target_link_libraries(io ${OpenCV_LIBS})

add_library(histogram_equalize histogram_equalize.cc)
target_link_libraries(histogram_equalize features ${OpenCV_LIBS})
add_test(histogram_equalize_test histogram_equalize_test.cc histogram_equalize ${OpenCV_LIBS})

add_library(images images.cc)

add_executable(main main.cc)
target_link_libraries(main features images histogram_equalize io ${OpenCV_LIBS})
