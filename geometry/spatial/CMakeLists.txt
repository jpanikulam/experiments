add_library(bounding_volume_hierarchy bounding_volume_hierarchy.cc)
add_executable(
    bounding_volume_hierarchy_test
    bounding_volume_hierarchy_test.cc
)
target_link_libraries(
    bounding_volume_hierarchy_test

    read_stl
    bounding_volume_hierarchy

    box
    simple_geometry
    viridis
    window_manager
    window_3d

    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}

    ${GTEST_LIBRARIES}
    pthread
)

add_library(octree octree.cc)
add_test(octree_test octree_test.cc "read_stl")

add_library(raycaster raycaster.cc)
target_link_libraries(
    raycaster
    ray
)
add_test(raycaster_test raycaster_test.cc read_stl)

add_test(bounding_box_test bounding_box_test.cc "")
