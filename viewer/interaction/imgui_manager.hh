#pragma once

#include <GLFW/glfw3.h>

namespace viewer {

//
// How to use:
//  1. Init (once) with a valid GLFWwindow
//
//  2. call new_frame()
//  3. Call ImGui commands and create an imgui
//  4. Call render() when done
//
// Note: You can use `want_capture()` to check if ImGui is
// trying to capture the keyboard or mouse
//

struct ImguiManagerConfig {
  bool opengl3 = false;
};

class ImGuiManager {
 public:
  void init(GLFWwindow* window, const ImguiManagerConfig& cfg = {});
  void new_frame();
  void render();
  bool want_capture() const;
  ~ImGuiManager();

 private:
  ImguiManagerConfig cfg_;
};

}  // namespace viewer
