#pragma once

//%deps(opengl)

#include "viewer/primitives/primitive.hh"

#include "eigen.hh"

#include <map>
#include <mutex>
#include <vector>

namespace viewer {

struct Surface {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Eigen::MatrixXd surface;
  double scale;
};

struct HistogramConfig {
  int num_bins;

  // If false, will automatically compute min/max
  bool force_min_max = false;
  // Max/min if force is enabled
  double min;
  double max;
};

struct OrderedPair {
  double x;
  double y;
};

struct SubLinePlot {
  std::vector<OrderedPair> points;
  jcc::Vec4 color = jcc::Vec4(1.0, 1.0, 1.0, 1.0);
  double line_width = 1.0;
  bool dotted = false;
};

struct LinePlot {
  std::string plot_title;
  std::string x_label;
  std::string y_label;
  std::map<std::string, SubLinePlot> subplots;
};

class Histogram {
 public:
  Histogram(const std::vector<double> &values,
            const HistogramConfig &config,
            const Eigen::Vector4d &color);

  void draw() const;

 private:
  HistogramConfig config_;
  std::vector<int> count_;
  double bin_size_;
  int max_count_;
  Eigen::Vector4d color_;
};

class Plot final : public Primitive {
 public:
  void draw() const override;

  void add_surface(const Surface &surface) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    surfaces_.push_back(surface);
  }

  void add_histogram(const Histogram &histogram) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    histograms_.push_back(histogram);
  }

  void add_line_plot(const LinePlot &line_plot) {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    line_plots_.push_back(line_plot);
  }

  void clear() {
    const std::lock_guard<std::mutex> lk(draw_mutex_);
    surfaces_.clear();
    histograms_.clear();
    line_plots_.clear();
  }

 private:
  // Will alignment fuck us?
  std::vector<Surface> surfaces_;
  std::vector<Histogram> histograms_;
  std::vector<LinePlot> line_plots_;

  mutable std::mutex draw_mutex_;
};
}  // namespace viewer
