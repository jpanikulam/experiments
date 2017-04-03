#pragma once

#include <map>
#include <string>

namespace gl_viewer {

//
// The window that you, the neighborhood dingus, get to manipulate
//
class SimpleWindow {
 public:
  // SimpleWindow();

  virtual void key_pressed(int key, int scancode, int action, int mods);

  virtual void on_key(int key, int scancode, int action, int mods) = 0;

  virtual void render() = 0;

  void set_text(const std::string text);

  const std::map<int, bool> &pressed_keys();

 private:
  std::string text_;
  std::map<int, bool> pressed_keys_;
};
}