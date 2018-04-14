add_test(numdiff_test numdiff_test.cc "")
add_test(covariance_test covariance_test.cc "")
add_test(gauss_newton_test gauss_newton_test.cc "")

add_test(conjugate_gradient_test conjugate_gradient_test.cc "")


add_library(
    gradient_descent
    gradient_descent.cc
)

add_test(gradient_descent_test gradient_descent_test.cc "gradient_descent")

add_executable(jacobi_laplacian_test jacobi_laplacian_test.cc)
target_link_libraries(jacobi_laplacian_test
    window_3d
    window_manager
    plot
    image
    frame
    image_frame
    simple_geometry
    ${OpenCV_LIBS}

    ${GTEST_LIBRARIES}
    pthread

)
