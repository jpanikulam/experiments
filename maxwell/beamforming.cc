#include "viewer/colors/colors.hh"
#include "viewer/interaction/ui2d.hh"
#include "viewer/primitives/simple_geometry.hh"

#include "viewer/window_3d.hh"

#include "eigen_helpers.hh"

namespace maxwell {
using Vec3 = Eigen::Vector3d;
using Complexd = std::complex<double>;

constexpr double C_MPS = 10.0;
constexpr double EMITTER_FREQ_HZ = 1000.0;

struct PointSource {
  Vec3 origin;
  double phase_rad;
  double amplitude;
};

Complexd sum_sources(const std::vector<PointSource>& sources, const Vec3& pt) {
  Complexd sum{0.0, 0.0};
  for (const auto& source : sources) {
    const double distance = (source.origin - pt).norm();
    const double true_phase_rad = source.phase_rad + (distance * 2.0 * M_PI / C_MPS);
    sum += source.amplitude * std::exp(Complexd(0.0, true_phase_rad));
  }
  return sum;
}

void render_beam(viewer::SimpleGeometry& geo, const std::vector<PointSource>& sources) {
  const auto view = viewer::get_window3d("Beams Baby");
  const double m_per_cell = 1.0;
  constexpr int GRID_SIZE = 50;
  constexpr int GRID_SIZE_Z = 50;
  constexpr int COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE_Z;

  viewer::Points points;
  std::vector<double> intensities;
  {
    points.points.reserve(COUNT);
    intensities.reserve(COUNT);
  }

  double max = 0.0;
  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      for (int k = -GRID_SIZE_Z; k < GRID_SIZE_Z; ++k) {
        const double x = m_per_cell * static_cast<double>(i);
        const double y = m_per_cell * static_cast<double>(j);
        const double z = m_per_cell * static_cast<double>(k);

        const Complexd total = sum_sources(sources, Vec3(x, y, z));

        double cell_intensity = 0.0;
        if (view->get_toggle("phase")) {
          cell_intensity = std::atan(total.imag() / total.real());
        } else {
          cell_intensity = std::abs(total);
        }

        max = std::max(cell_intensity, max);
        points.points.push_back(Vec3(x, y, z));
        intensities.push_back(cell_intensity);
      }
    }
  }

  for (const auto& source : sources) {
    const Vec3 half_z = 0.5 * Vec3::UnitZ() * (source.phase_rad / 10.0);
    geo.add_line({source.origin - half_z, source.origin + half_z});
  }

  geo.add_colored_points(points, intensities);
  geo.flip();
}

void beamform() {
  const auto view = viewer::get_window3d("Beams Baby");
  view->add_toggle_hotkey("phase", false, 'P');

  const auto bgnd = view->add_primitive<viewer::SimpleGeometry>();
  const geometry::shapes::Plane ground{Vec3::UnitZ(), 0.0};
  bgnd->add_plane({ground});
  bgnd->flip();

  const auto demo_geo = view->add_primitive<viewer::SimpleGeometry>();
  const auto demo_text = view->add_primitive<viewer::Ui2d>();

  const auto hover_callback = [demo_geo, demo_text](int a,
                                                    double dist_sq,
                                                    const Vec3& ray_pt,
                                                    const Vec3& p,
                                                    const viewer::ViewportPoint& vp) {
    if (dist_sq < 1.0) {
      demo_geo->add_sphere({ray_pt, 0.1, viewer::red()});
      demo_geo->add_line({ray_pt, p, viewer::red()});
      demo_text->add_pointer_target({"Dist: " + std::to_string(dist_sq), p, vp});
    }
    demo_text->flip();
    demo_geo->flip();
  };

  view->add_click_callback(
      hover_callback,
      {geometry::shapes::LineSegment{Vec3(1.0, 1.0, 1.0), Vec3(0.0, 0.0, 10.0)}});

  // view->add_click_callback(
  //     hover_callback,
  //     geometry::Plane{Vec3(0.0, 0.0, 0.0), geometry::Unit3(0.0, 0.0, 1.0)});

  bgnd->add_line({jcc::Vec3(1.0, 1.0, 1.0), jcc::Vec3(0.0, 0.0, 10.0),
                  jcc::Vec4(1.0, 0.5, 0.5, 0.8)});
  bgnd->flip();

  const auto geo = view->add_primitive<viewer::SimpleGeometry>();

  constexpr double MAX_SCALE = 100.0;
  constexpr double COUNT = 1000.0;

  const Vec3 dish_origin(1.0, 25.0, 0.0);
  for (double offset = 0.0; offset < MAX_SCALE; offset += (MAX_SCALE / COUNT)) {
    std::vector<PointSource> sources;
    int emitter_count = 6;
    for (int k = 0; k < emitter_count; ++k) {
      // Two interesting reciever patterns:
      // Linear arrangement with linear phase
      // Parabolic arrangement with linear phase, or probably some abs phase

      const int emitter_num = k - 3;
      const double phase = emitter_num * offset;
      const double xx = 0.8 * emitter_num;
      const Vec3 p((xx * xx), emitter_num * 5.0, 0.0);

      sources.push_back({p + dish_origin, phase, 1.0 / emitter_count});
    }

    for (int k = 0; k < emitter_count; ++k) {
      const int emitter_num = k - 3;
      const double phase = emitter_num * offset;
      const double xx = 0.8 * emitter_num;
      const Vec3 p((xx * xx), 0.0, emitter_num * 5.0);

      sources.push_back({p + dish_origin, phase, 1.0 / emitter_count});
    }

    view->clear_toggle_callbacks("phase");
    view->add_toggle_callback("phase",
                              [geo, sources](bool) { render_beam(*geo, sources); });

    render_beam(*geo, sources);
    view->spin_until_step();
  }
}

}  // namespace maxwell

int main() {
  maxwell::beamform();
}
