#pragma once

#include "viewer/gl_types.hh"
#include "viewer/primitives/primitive.hh"
#include "viewer/text/gl_text.hh"

#include <mutex>
#include <vector>

namespace viewer {

struct PointerTarget {
  std::string text;
  jcc::Vec3 world_pos;
  ViewportPoint location;
};

struct SubLinePlot2d {
  std::vector<jcc::Vec2> points;
  jcc::Vec4 color = jcc::Vec4(1.0, 1.0, 1.0, 1.0);
  double line_width = 1.0;
  bool dotted = false;
};

struct LinePlot2d {
  std::string plot_title;
  std::string x_label;
  std::string y_label;
  std::map<std::string, SubLinePlot2d> subplots;
};

class SubLinePlotBuilder {
 public:
  SubLinePlotBuilder() = default;
  SubLinePlotBuilder(const jcc::Vec4& color) {
    subplot_.color = color;
  }
  void operator<<(const jcc::Vec2& p) {
    subplot_.points.push_back(p);
  }

  SubLinePlot2d build() const {
    return subplot_;
  }

 private:
  SubLinePlot2d subplot_;
};

class LinePlotBuilder {
 public:
  LinePlotBuilder(const std::string& plot_title,
                  const std::string& x_label = "",
                  const std::string& y_label = "") {
    plot_.plot_title = plot_title;
    plot_.x_label = x_label;
    plot_.y_label = y_label;
  }
  SubLinePlotBuilder& make_subplot(const std::string& name, const jcc::Vec4& color) {
    builders_[name] = SubLinePlotBuilder(color);
    return builders_.at(name);
  }

  LinePlot2d build() const {
    LinePlot2d plot = plot_;
    for (const auto& subplot_builder : builders_) {
      plot.subplots[subplot_builder.first] = subplot_builder.second.build();
    }
    return plot;
  }

 private:
  std::map<std::string, SubLinePlotBuilder> builders_;
  LinePlot2d plot_;
};

class Ui2d final : public Primitive {
 public:
  Ui2d() = default;
  void draw() const override;

  void add_pointer_target(const PointerTarget& pointer_target);
  void add_lineplot(const LinePlot2d& line_plot);
  void add_lineplot(const LinePlotBuilder& line_plot) {
    add_lineplot(line_plot.build());
  }

  void clear();

  void flush();
  void flip();

 private:
  struct Buffer {
    std::vector<PointerTarget> pointer_targets;
    std::vector<LinePlot2d> line_plots;

    void clear() {
      pointer_targets.clear();
      line_plots.clear();
    }
  };

  mutable CharacterLibrary char_lib_;

  Buffer back_buffer_;
  Buffer front_buffer_;

  mutable std::mutex draw_mutex_;
};

}  // namespace viewer
