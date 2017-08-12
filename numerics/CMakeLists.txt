cmake_minimum_required(VERSION 2.8)

#########################
# Eigen
#
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported)

set(BASEPATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
INCLUDE_DIRECTORIES("${BASEPATH}")

# add_executable(
#     lu
#     lu.cc
# )
# target_link_libraries(lu)

# add_library(numdiff numdiff.cc)
add_test(numdiff_test numdiff_test.cc "")
add_test(covariance_test covariance_test.cc "")

add_test(conjugate_gradient_test conjugate_gradient_test.cc "")
