#pragma once

#include "viewer/gl_types.hh"
#include "viewer/interaction/plot_builder.hh"
#include "viewer/primitives/primitive.hh"
#include "viewer/rendering/smart_texture.hh"
#include "viewer/text/gl_text.hh"

//%deps(opencv, opengl)
#include <opencv2/opencv.hpp>

#include <mutex>
#include <vector>

namespace viewer {

struct PointerTarget {
  std::string text;
  jcc::Vec3 world_pos;
  ViewportPoint location;
};

struct Image {
  // You cannot imagine how much I hate this.
  cv::Mat image;
  double width_m;
  mutable SmartTexture texture;
};

class Ui2d final : public Primitive {
 public:
  Ui2d() = default;
  void draw() const override;

  void add_pointer_target(const PointerTarget& pointer_target);
  void add_lineplot(const LinePlot2d& line_plot);
  void add_lineplot(const LinePlotBuilder& line_plot_builder) {
    add_lineplot(line_plot_builder.build());
  }

  void add_image(const cv::Mat& image, double width_m) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    back_buffer_.images.push_back({image, width_m});
  }

  void clear();
  void flush();
  void flip();

 private:
  struct Buffer {
    std::vector<PointerTarget> pointer_targets;
    std::vector<LinePlot2d> line_plots;
    std::vector<Image> images;

    void clear() {
      pointer_targets.clear();
      line_plots.clear();
      images.clear();
    }
  };

  mutable CharacterLibrary char_lib_;

  Buffer back_buffer_;
  Buffer front_buffer_;

  mutable std::mutex draw_mutex_;
};

}  // namespace viewer
