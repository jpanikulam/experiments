#include "viewer/interaction/plot_builder.hh"

namespace viewer {

SubLinePlotBuilder::SubLinePlotBuilder(const jcc::Vec4& color,
                                       const double width,
                                       const bool dotted) {
  subplot_.color = color;
  subplot_.line_width = width;
  subplot_.dotted = dotted;
}

void SubLinePlotBuilder::operator<<(const jcc::Vec2& p) {
  subplot_.points.push_back(p);
}

SubLinePlot2d SubLinePlotBuilder::build() const {
  return subplot_;
}

LinePlotBuilder::LinePlotBuilder(const std::string& plot_title,
                                 const std::string& x_label,
                                 const std::string& y_label) {
  plot_.plot_title = plot_title;
  plot_.x_label = x_label;
  plot_.y_label = y_label;
}

void LinePlotBuilder::set_range(const PlotRange& range) {
  plot_.plot_range = range;
}

SubLinePlotBuilder& LinePlotBuilder::make_subplot(const std::string& name,
                                                  const jcc::Vec4& color,
                                                  const double line_width,
                                                  const bool dotted) {
  builders_[name] = SubLinePlotBuilder(color, line_width, dotted);
  return builders_.at(name);
}

LinePlot2d LinePlotBuilder::build() const {
  LinePlot2d plot = plot_;
  for (const auto& subplot_builder : builders_) {
    plot.subplots[subplot_builder.first] = subplot_builder.second.build();
  }
  return plot;
}

}  // namespace viewer