#pragma once

#include "viewer/gl_size.hh"
#include "viewer/gl_types.hh"

#include "macros.hh"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <string>
#include <unordered_map>

namespace viewer {

//
// The window that you, the neighborhood dingus, get to manipulate
//
class SimpleWindow {
 public:
  virtual ~SimpleWindow() = default;

  void maybe_init() {
    if (UNLIKELY(!initialized_)) {
      init(gl_size_);
      initialized_ = true;
    }
  }

  virtual void init(const GlSize &gl_size) {
  }

  virtual void key_pressed(int key, int scancode, int action, int mods);
  virtual void mouse_button(int button, int action, int mods);
  virtual void mouse_moved(double x, double y);

  virtual void on_key(int key, int scancode, int action, int mods) {
  }

  virtual void on_mouse_button(int button, int action, int mods) {
  }

  virtual void on_mouse_move(const WindowPoint &pos) {
  }

  virtual void on_scroll(const double amount) {
  }

  virtual void resize(const GlSize &gl_size);

  virtual void render() = 0;

  void set_size(const GlSize &size) {
    gl_size_ = size;
  }

  virtual void close() {
    should_close_ = true;
  };

  virtual bool should_close() const {
    return should_close_;
  }

  void set_window(GLFWwindow *window) {
    window_ = window;
  }

  GLFWwindow *get_window() {
    return window_;
  }

  // Make the title something
  void set_title(const std::string title);

  // Get the title
  const std::string &title() const;

  const std::unordered_map<int, bool> &held_keys() const;
  const std::array<bool, 3> &held_mouse_buttons() const;

  const WindowPoint &mouse_pos() const;

  bool left_mouse_held() const;
  bool right_mouse_held() const;

  const GlSize &gl_size() const {
    return gl_size_;
  }

 private:
  bool initialized_ = false;

  std::string title_;
  std::unordered_map<int, bool> held_keys_;
  std::array<bool, 3> held_mouse_buttons_ = {false, false, false};
  bool should_close_ = false;

  GlSize gl_size_;
  WindowPoint mouse_pos_;

  GLFWwindow *window_;
};
}  // namespace viewer
