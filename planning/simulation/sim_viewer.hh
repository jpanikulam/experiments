#include "viewer/window_3d.hh"

namespace jcc {

class SimViewer : public Window3D {
 public:
  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void draw() override;
};

std::shared_ptr<SimViewer> create_sim_viewer(const std::string &title) {
  const GlSize gl_size(640, 640);
  auto window = std::make_shared<SimViewer>(gl_size);
  get_global_state().windows[title] = window;
  WindowManager::register_window(gl_size, window, title);
  return window;
}

}  // namespace jcc