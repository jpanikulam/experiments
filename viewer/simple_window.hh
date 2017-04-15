#pragma once

#include "gl_size.hh"
#include "gl_types.hh"

#include <map>
#include <string>

namespace gl_viewer {

//
// The window that you, the neighborhood dingus, get to manipulate
//
class SimpleWindow {
 public:
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

  virtual void resize(const GlSize &gl_size) {
  }

  virtual void render() = 0;

  void set_title(const std::string title);
  const std::string &title();

  const std::map<int, bool> &held_keys() const;
  const std::map<int, bool> &held_mouse_buttons() const;

  const WindowPoint &mouse_pos() const;

  bool left_mouse_held() const;
  bool right_mouse_held() const;

 private:
  std::string title_;
  std::map<int, bool> held_keys_;
  std::map<int, bool> held_mouse_buttons_;

  WindowPoint mouse_pos_;
};
}