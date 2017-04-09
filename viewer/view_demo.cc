#include "window_2d.hh"
#include "window_manager.hh"

#include <iostream>
#include <memory>
#include <thread>

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

namespace gl_viewer {

void run() {
  WindowManager win_man;
  auto          win2d = std::make_shared<Window2D>();
  win_man.register_window(GlSize(640, 640), win2d, "Two Dimensional Debugging");

  const Vec2 v(0.0, 1.0);
  win2d->add_line({Vec2(-0.5, -0.5), Vec2(0.5, 0.5)});
  win2d->add_line({Vec2(-0.5, 0.5), Vec2(0.5, -0.5)});

  win2d->add_ray({Vec2(0.0, 0.0), Vec2(-0.5, 0.5), Vec4(0.0, 1.0, 0.4, 1.0)});

  while (win_man.any_windows()) {
    win_man.render();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
  }
  std::cout << "Done" << std::endl;

  glfwTerminate();
  exit(EXIT_SUCCESS);
}
}

int main(void) {
  gl_viewer::run();
}