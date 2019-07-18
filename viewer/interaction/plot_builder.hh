#pragma once

#include "eigen.hh"

#include <map>
#include <string>
#include <vector>

namespace viewer {

enum class LinePlotStyle {
  Line = 0,
  Scatter = 1,
};

struct SubLinePlot2d {
  std::vector<jcc::Vec2> points;
  jcc::Vec4 color = jcc::Vec4(1.0, 1.0, 1.0, 1.0);
  double line_width = 1.0;
  bool dotted = false;
  LinePlotStyle style = LinePlotStyle::Line;
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
                     const bool dotted = false,
                     const LinePlotStyle style = LinePlotStyle::Line);
  void operator<<(const jcc::Vec2& p);

  SubLinePlot2d build() const;

 private:
  SubLinePlot2d subplot_;
};

class LinePlotBuilder {
 public:
  LinePlotBuilder(const std::string& plot_title,
                  const std::string& x_label = "",
                  const std::string& y_label = "");

  void set_range(const PlotRange& range);

  SubLinePlotBuilder& make_subplot(const std::string& name,
                                   const jcc::Vec4& color,
                                   const double width = 0.2,
                                   const bool dotted = false,
                                   const LinePlotStyle style = LinePlotStyle::Line);

  LinePlot2d build() const;

 private:
  std::map<std::string, SubLinePlotBuilder> builders_;
  LinePlot2d plot_;
};
}  // namespace viewer