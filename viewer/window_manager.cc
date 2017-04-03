#include "window_manager.hh"

#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include <stdlib.h>

#define GLSL(src) #src

namespace gl_viewer {

struct GlobalState {
  GLFWwindow *active_window;
  std::map<GLFWwindow *, std::shared_ptr<SimpleWindow>> windows;
};

GlobalState *global_state;

GlobalState *maybe_create_global_state() {
  if (!global_state) {
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
      exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

    glewExperimental = GL_TRUE;
    glewInit();

    global_state = new GlobalState;
  }
  return global_state;
}

/*
std::shared_ptr<SimpleWindow> WindowManager::create_window(const GlSize &size) {
  maybe_create_global_state();

  std::ostringstream win_name;
  win_name << "Window : ";
  win_name << global_state->windows.size();
  auto simple_window = std::make_shared<SimpleWindow>();
  simple_window->set_text(win_name.str());

  register_window(size, simple_window, win_name.str());

  return simple_window;
}
*/

void WindowManager::register_window(const GlSize &                      size,
                                    const std::shared_ptr<SimpleWindow> simple_window,
                                    const std::string &                 window_name) {
  maybe_create_global_state();

  GLFWwindow *window = glfwCreateWindow(size.height, size.width, window_name.c_str(), nullptr, nullptr);

  if (!window) {
    glfwTerminate();
    std::cerr << "Failed to create new window" << std::endl;
    exit(EXIT_FAILURE);
  }

  glfwSetKeyCallback(window, key_callback);
  global_state->windows[window] = simple_window;
}

//
// Render all of the managed windows
//
void WindowManager::render() {
  for (auto it = global_state->windows.begin(); it != global_state->windows.end(); it++) {
    auto &glfw_win = it->first;
    auto &window   = it->second;

    glfwMakeContextCurrent(glfw_win);

    if (!glfwWindowShouldClose(glfw_win)) {
      window->render();
      glfwSwapBuffers(glfw_win);

    } else {
      glfwDestroyWindow(glfw_win);
      global_state->windows.erase(it);
      continue;
    }
  }
  glfwPollEvents();
}

bool WindowManager::any_windows() {
  maybe_create_global_state();
  return !global_state->windows.empty();
}

void error_callback(int error, const char *description) {
  fputs(description, stderr);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  if (key == GLFW_KEY_Q) {
    glfwSetWindowShouldClose(window, GL_TRUE);
  }

  global_state->windows.at(window)->key_pressed(key, scancode, action, mods);
}
}