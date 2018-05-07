add_library(
    fast_json
    fast_json.cc
)

add_executable(
    visualize_geojson
    visualize_geojson.cc
)

target_link_libraries(
    visualize_geojson

    fast_json
    window_3d
    simple_geometry
)
