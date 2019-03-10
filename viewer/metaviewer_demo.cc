#include "viewer/metaviewer.hh"

#include "viewer/primitives/plot.hh"

viewer::LinePlot generate_line_plot(int n) {
  viewer::LinePlot line_plot;
  for (double x = 0.0; x < 10.0; x += 0.01) {
    const double y = n * jcc::Vec1::Random()[0];
    line_plot.subplots["bonanza"].points.push_back({x, y});
  }
  return line_plot;
}

int main() {
  const auto meta_view = viewer::get_metaviewer();

  const auto win3d_0 = meta_view->add_window3d();
  const auto plot_0 = win3d_0->add_primitive<viewer::Plot>();
  const auto line_plot_0 = generate_line_plot(1);
  plot_0->add_line_plot(line_plot_0);

  // const auto win3d_1 = meta_view->add_window3d();
  // const auto plot_1 = win3d_1->add_primitive<viewer::Plot>();
  // const auto line_plot_1 = generate_line_plot(2);
  // plot_1->add_line_plot(line_plot_1);
  // win3d_1->spin_until_step();

  win3d_0->spin_until_step();

  return 0;
}
