#pragma once

#include "viewer/gl_size.hh"
#include "viewer/projection.hh"
#include "viewer/simple_window.hh"
#include "viewer/window_3d.hh"

#include "eigen.hh"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

namespace viewer {

class MetaViewer final : public SimpleWindow {
 public:
  MetaViewer(const GlSize &gl_size) : gl_size_(gl_size){};

  void on_key(int key, int scancode, int action, int mods) override;
  void on_mouse_button(int button, int action, int mods) override;
  void on_mouse_move(const WindowPoint &mouse_pos) override;
  void on_scroll(const double amount) override;

  void render() override;

  void resize(const GlSize &gl_size) override;

  std::shared_ptr<Window3D> add_window3d(const std::string &title = "main") {
    const GlSize gl_size(640, 320);
    const auto window = std::make_shared<Window3D>(gl_size);
    window->set_window(get_window());
    partitions_.push_back(window);
    return window;
  }

  void clear() {
    const std::lock_guard<std::mutex> lk(behavior_mutex_);
    partitions_.clear();
  }

 private:
  // PartitionStructure partitions_;
  std::vector<std::shared_ptr<SimpleWindow>> partitions_;

  GlSize gl_size_;

  //
  // UI Control State
  //

  // Can potentially get rid of this
  WindowPoint mouse_pos_last_click_;

  //
  // Interaction History
  //

  double last_update_time_ = 0.0;

  mutable std::mutex behavior_mutex_;
};

std::shared_ptr<MetaViewer> get_metaviewer(const std::string &title = "main");
}  // namespace viewer
