#include "window_manager.hh"

#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>

#define GLSL(src) #src

namespace gl_viewer {

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void window_size_callback(GLFWwindow *window, int width, int height);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void error_callback(int error, const char *description);

//
// Manage a global registry of the created windows
//

struct GlobalState {
  GLFWwindow *active_window;
  std::map<GLFWwindow *, std::shared_ptr<SimpleWindow>> windows;
};

// Global state singleton
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

void WindowManager::register_window(const GlSize &                      size,
                                    const std::shared_ptr<SimpleWindow> simple_window,
                                    const std::string &                 window_name) {
  maybe_create_global_state();

  GLFWwindow *window = glfwCreateWindow(size.height, size.width, window_name.c_str(), nullptr, nullptr);
  simple_window->set_title(window_name);

  if (!window) {
    glfwTerminate();
    std::cerr << "Failed to create new window" << std::endl;
    exit(EXIT_FAILURE);
  }

  glfwSetKeyCallback(window, key_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  global_state->windows[window] = simple_window;
  draw();
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

void WindowManager::draw(const int ms) {
  int ms_slept = 0;
  while (any_windows() && (ms_slept < ms)) {
    constexpr int SLEEP_MS = 2;
    render();
    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    ms_slept += SLEEP_MS;
  }
}

void WindowManager::spin() {
  while (any_windows()) {
    render();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  glfwTerminate();
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

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
  global_state->windows.at(window)->mouse_moved(xpos, ypos);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
  }

  if (action == GLFW_RELEASE) {
  }

  global_state->windows.at(window)->mouse_button(button, action, mods);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  global_state->windows.at(window)->on_scroll(yoffset);
}

void window_size_callback(GLFWwindow *window, int width, int height) {
  global_state->windows.at(window)->resize(GlSize(width, height));
}
}
