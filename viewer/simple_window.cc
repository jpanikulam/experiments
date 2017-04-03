#include "simple_window.hh"

#include <GLFW/glfw3.h>

#include <map>
#include <string>

// todo
#include <iostream>

namespace gl_viewer {

void SimpleWindow::key_pressed(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    pressed_keys_[key] = true;
  } else if (action == GLFW_RELEASE) {
    pressed_keys_[key] = false;
  }

  on_key(key, scancode, action, mods);
}

void on_key(int key, int scancode, int action, int mods) {
}

void SimpleWindow::set_text(const std::string text) {
  text_ = text;
}

const std::map<int, bool> &SimpleWindow::pressed_keys() {
  return pressed_keys_;
}
}