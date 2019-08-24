#include "viewer/interaction/imgui_manager.hh"

#include "third_party/imgui/imgui.h"

#include "third_party/imgui/examples/imgui_impl_glfw.h"
#include "third_party/imgui/examples/imgui_impl_opengl2.h"

// %deps(imgui)

namespace viewer {

void ImGuiManager::init(GLFWwindow* window) {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  // TODO(jpanikulam): Handle multiple windows
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard
  // Controls io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable
  // Gamepad Controls

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  constexpr bool INSTALL_CALLBACKS = true;
  ImGui_ImplGlfw_InitForOpenGL(window, INSTALL_CALLBACKS);
  ImGui_ImplOpenGL2_Init();
}

bool ImGuiManager::want_capture() const {
  const ImGuiIO& io = ImGui::GetIO();
  return io.WantCaptureMouse || io.WantCaptureKeyboard;
}

void ImGuiManager::new_frame() {
  // Start the Dear ImGui frame
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void ImGuiManager::render() {
  ImGui::Render();
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

ImGuiManager::~ImGuiManager() {
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

}  // namespace viewer
