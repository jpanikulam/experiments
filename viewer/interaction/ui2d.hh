#pragma once

#include "util/optional.hh"
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

struct PlotRange {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};
struct LinePlot2d {
  std::string plot_title;
  std::string x_label;
  std::string y_label;
  std::map<std::string, SubLinePlot2d> subplots;
  PlotRange plot_range;
};

class SubLinePlotBuilder {
 public:
  SubLinePlotBuilder() = default;
  SubLinePlotBuilder(const jcc::Vec4& color,
                     const double width = 0.2,
                     const bool dotted = false) {
    subplot_.color = color;
    subplot_.line_width = width;
    subplot_.dotted = dotted;
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

  void set_range(const PlotRange& range) {
    plot_.plot_range = range;
  }

  SubLinePlotBuilder& make_subplot(const std::string& name,
                                   const jcc::Vec4& color,
                                   const double line_width = 0.2,
                                   const bool dotted = false) {
    builders_[name] = SubLinePlotBuilder(color, line_width, dotted);
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
  void add_lineplot(const LinePlotBuilder& line_plot_builder) {
    add_lineplot(line_plot_builder.build());
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
