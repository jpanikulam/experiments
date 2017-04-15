#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "gl_size.hh"
#include "simple_window.hh"  // Can't include before glew

#include <memory>

namespace gl_viewer {

//
// Callback router
//
class WindowManager {
 public:
  // Tool for managing windows

  // Create and register windows with the manager
  //
  void register_window(const GlSize &size, const std::shared_ptr<SimpleWindow> win, const std::string &window_name);

  // Render all of the managed windows
  //
  void render();

  // Are there any active windows?
  //
  bool any_windows();

  // Block and run until an exit is requested
  void spin();

  void draw(const int ms = 16);
};
}