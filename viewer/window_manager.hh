#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "simple_window.hh"  // Can't include before glew

#include <memory>

namespace gl_viewer {

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void error_callback(int error, const char *description);

struct GlSize {
  GlSize() {
  }
  GlSize(int h, int w) {
    height = h;
    width  = w;
  }

  int height = 640;
  int width  = 480;
};

//
// Callback router
//
class WindowManager {
 public:
  //
  // Create and register windows with the manager
  //
  // std::shared_ptr<SimpleWindow> create_window(const GlSize &size);

  void register_window(const GlSize &size, const std::shared_ptr<SimpleWindow> win, const std::string &window_name);

  //
  // Render all of the managed windows
  //
  void render();

  //
  // Are there any active windows?
  //
  bool any_windows();
};
}