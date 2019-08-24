add_library(imgui
	examples/imgui_impl_glfw.cpp
	examples/imgui_impl_opengl2.cpp

    examples/example_glfw_opengl2/main.cpp
    examples/imgui_impl_glfw.cpp
    examples/imgui_impl_opengl2.cpp

    imgui.cpp
    imgui_draw.cpp
    imgui_widgets.cpp
)
target_link_libraries(imgui ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})



include_directories("${BASEPATH}/third_party/imgui")

add_executable(
    imgui_demo

    examples/example_glfw_opengl2/main.cpp
    imgui_demo.cpp
)

target_link_libraries(imgui_demo imgui ${OPENGL_LIBRARIES} ${GLFW_LIBRARIES})
