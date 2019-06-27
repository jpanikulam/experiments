#include "planning/body.hh"

#include "logging/assert.hh"
#include "numerics/numdiff.hh"

#include "viewer/primitives/simple_geometry.hh"
#include "viewer/window_3d.hh"

//
// - Fix yaw
//    - Draw a uniformly sampled (in output) yaw transfer
//    - Write golden-section method + quadratic interpolation linesearch

namespace planning {

jcc::Vec3 immersion(const jcc::Vec2& uv) {
  const double x = uv.x() * uv.x();
  const double y = uv.y() * uv.y() * uv.y();
  const double z = std::sqrt(x);
  const jcc::Vec3 xyz(x, y, z);
  return xyz;
}

const jcc::Vec3 df(const jcc::Vec2& uv, const jcc::Vec2& du) {
  const MatNd<3, 2> J = numerics::numerical_jacobian<3>(uv, immersion);
  const jcc::Vec3 push_fwd = J * du;
  return push_fwd;
}

double length_scaling(const jcc::Vec2& uv, const jcc::Vec2& du) {
  const jcc::Vec3 push_fwd = df(uv, du);
  const double w_du = std::sqrt(push_fwd.dot(push_fwd));
  return w_du;
}

double theta_immersion(const jcc::Vec2& uv, const double phi) {
  const jcc::Vec2 surface_heading(std::cos(phi), std::sin(phi));

  const jcc::Vec3 df_xhat = df(uv, jcc::Vec2::UnitX());
  const jcc::Vec3 df_dirhat = df(uv, surface_heading);

  const double the_dot_part = df_xhat.normalized().dot(df_dirhat.normalized());
  const double cos_theta = the_dot_part;
  const double theta = std::acos(cos_theta);
  return theta;
}

double theta_scaling(const jcc::Vec2& uv, const double phi, const double dphi) {
  const auto theta_of_phi = [uv](const jcc::Vec1& phi_arg) {
    return theta_immersion(uv, phi_arg[0]);
  };

  const auto J = numerics::numerical_gradient(jcc::Vec1(phi), theta_of_phi);
  const double dtheta = J[0] * dphi;

  // Identity, I know!
  // return std::sqrt(dtheta * dtheta);
  return dtheta;
}

std::vector<double> uniform_sample(const double start,
                                   const double end,
                                   const std::function<double(double)>& fnc) {
  double x = start;
  double prev_x = x;

  const auto dfnc = [&fnc](double xx) {
    constexpr double EPS = 1e-7;
    constexpr double INV_2_EPS = 1.0 / (2.0 * EPS);
    const double error = fnc(xx + EPS) - fnc(xx - EPS);
    return error / INV_2_EPS;
  };

  // Must be monotonic
  // while (x > prev_x) {
  //   while (std::abs(fnc(x) - fnc(prev_x)) <) {
  //   }
  // }
}

void go() {
  const auto view = viewer::get_window3d("Mr. Arm, arms");

  const auto surf_geo = view->add_primitive<viewer::SimpleGeometry>();

  constexpr double SPACING = 0.1;
  for (double u = 0.0; u < 5.0; u += SPACING) {
    for (double v = 0.0; v < 5.0; v += SPACING) {
      const jcc::Vec2 uv(u, v);
      const jcc::Vec3 f_uv = immersion(uv);

      const jcc::Vec2 du = jcc::Vec2::UnitX();
      const jcc::Vec2 dv = jcc::Vec2::UnitY();

      const MatNd<3, 2> J = numerics::numerical_jacobian<3>(uv, immersion);
      const jcc::Vec3 push_fwd_du = J * du;
      const jcc::Vec3 push_fwd_dv = J * dv;
      const double w_x = std::sqrt(push_fwd_du.dot(push_fwd_du));
      const double w_y = std::sqrt(push_fwd_dv.dot(push_fwd_dv));

      surf_geo->add_line({f_uv, f_uv + (SPACING * push_fwd_du.normalized() * w_x),
                          jcc::Vec4(1.0, 0.0, 0.0, 1.0)});
      surf_geo->add_line({f_uv, f_uv + (SPACING * push_fwd_dv.normalized() * w_y),
                          jcc::Vec4(0.0, 1.0, 0.0, 1.0)});

      surf_geo->add_point({f_uv, jcc::Vec4(1.0, 1.0, 0.0, 1.0), 5.0});
    }
  }

  surf_geo->flip();

  const auto test_geo = view->add_primitive<viewer::SimpleGeometry>();

  const jcc::Vec2 test_uv(2.5, 2.5);

  /*const double dphi = 0.1;
  for (double phi = 0.2; phi < 2.0 * M_PI;
       phi += dphi / theta_scaling(test_uv, phi, dphi)) {
    std::cout << "dphi: " << dphi / theta_scaling(test_uv, phi, dphi) << std::endl;
    const jcc::Vec2 direction(std::cos(phi), std::sin(phi));
    const jcc::Vec3 push_direction = df(test_uv, direction).normalized();

    test_geo->add_ray({immersion(test_uv), push_direction});
  }*/

  /*  const double dphi = 0.1;
    for (double phi = 0.2; phi < 2.0 * M_PI; phi += dphi) {
      const jcc::Vec2 direction(std::cos(phi), std::sin(phi));
      const jcc::Vec3 push_direction = df(test_uv, direction).normalized();
      // test_geo->add_ray({immersion(test_uv), push_direction});
      const double theta = theta_immersion(test_uv, phi);
      test_geo->add_point({jcc::Vec3(phi, 0.0, theta)});
    }
  */
  test_geo->flip();

  view->spin_until_step();
}
}  // namespace planning

int main() {
  planning::go();
}