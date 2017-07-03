add_library(bootstrap_pose bootstrap_pose.cc)
target_link_libraries(bootstrap_pose ${OpenCV_LIBS})
add_test(bootstrap_pose_test bootstrap_pose_test.cc bootstrap_pose ${OpenCV_LIBS})
