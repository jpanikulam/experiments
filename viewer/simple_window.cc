#include "simple_window.hh"

#include <GLFW/glfw3.h>

#include <map>
#include <string>

namespace gl_viewer {

void SimpleWindow::key_pressed(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    held_keys_[key] = true;
  } else if (action == GLFW_RELEASE) {
    held_keys_[key] = false;
  }

  on_key(key, scancode, action, mods);
}

void SimpleWindow::mouse_button(int button, int action, int mods) {
  if (action == GLFW_PRESS) {
    held_mouse_buttons_[button] = true;
  } else if (action == GLFW_RELEASE) {
    held_mouse_buttons_[button] = false;
  }

  on_mouse_button(button, action, mods);
}

void SimpleWindow::mouse_moved(double x, double y) {
  mouse_pos_.point(0) = x;
  mouse_pos_.point(1) = y;
  on_mouse_move(mouse_pos_);
}

bool SimpleWindow::left_mouse_held() const {
  const auto search = held_mouse_buttons_.find(GLFW_MOUSE_BUTTON_LEFT);
  if (search != held_mouse_buttons_.end()) {
    return search->second;
  } else {
    return false;
  }
}

bool SimpleWindow::right_mouse_held() const {
  const auto search = held_mouse_buttons_.find(GLFW_MOUSE_BUTTON_RIGHT);
  if (search != held_mouse_buttons_.end()) {
    return search->second;
  } else {
    return false;
  }
}

void SimpleWindow::set_title(const std::string title) {
  title_ = title;
}

const std::string &SimpleWindow::title() {
  return title_;
}

const std::map<int, bool> &SimpleWindow::held_keys() const {
  return held_keys_;
}

const std::map<int, bool> &SimpleWindow::held_mouse_buttons() const {
  return held_mouse_buttons_;
}

const WindowPoint &SimpleWindow::mouse_pos() const {
  return mouse_pos_;
}
}
