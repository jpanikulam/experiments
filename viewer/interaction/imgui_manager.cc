#include "viewer/interaction/imgui_manager.hh"

#include "third_party/imgui/imgui.h"

#include "third_party/imgui/examples/imgui_impl_glfw.h"
#include "third_party/imgui/examples/imgui_impl_opengl2.h"
#include "third_party/imgui/examples/imgui_impl_opengl3.h"

//TODO
#include <iostream>

// %deps(imgui)

namespace viewer {

void ImGuiManager::init(GLFWwindow* window, const ImguiManagerConfig& cfg) {
  cfg_ = cfg;
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  // TODO(jpanikulam): Handle multiple windows

  std::cout << "Version: " << cfg.opengl3 << std::endl;

  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  constexpr bool INSTALL_CALLBACKS = true;
  ImGui_ImplGlfw_InitForOpenGL(window, INSTALL_CALLBACKS);
  if (cfg_.opengl3) {
    ImGui_ImplOpenGL3_Init();
  } else {
    ImGui_ImplOpenGL2_Init();
  }
}

bool ImGuiManager::want_capture() const {
  const ImGuiIO& io = ImGui::GetIO();
  return io.WantCaptureMouse || io.WantCaptureKeyboard;
}

void ImGuiManager::new_frame() {
  // Start the Dear ImGui frame
  if (cfg_.opengl3) {
    ImGui_ImplOpenGL3_NewFrame();
  } else {
    ImGui_ImplOpenGL2_NewFrame();
  }
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void ImGuiManager::render() {
  ImGui::Render();
  if (cfg_.opengl3) {
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  } else {
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
  }
}

ImGuiManager::~ImGuiManager() {
  if (cfg_.opengl3) {
    ImGui_ImplOpenGL3_Shutdown();
  } else {
    ImGui_ImplOpenGL2_Shutdown();
  }
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

}  // namespace viewer
