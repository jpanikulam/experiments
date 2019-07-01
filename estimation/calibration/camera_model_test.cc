#include "estimation/calibration/camera_model.hh"

#include "testing/gtest.hh"

namespace estimation {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

TEST(CameraModel, project_inverts_unproject) {
  //
  // Setup
  //

  const ProjectionCoefficients coeffs{.fx = 999.2, .fy = 1000.1, .cx = 0.0, .cy = 0.0};
  const CameraModel model(coeffs);

  //
  // Action / Verification
  //

  constexpr double EPS = 1e-6;
  {
    const Vec2 pt(630, 290);
    const geometry::Ray ray = model.unproject(pt);
    const Vec2 projected = model.project(ray.direction);

    EXPECT_LT((pt - projected).norm(), EPS);
  }

  {
    const Vec2 pt(12, 82);
    const geometry::Ray ray = model.unproject(pt);
    const Vec2 projected = model.project(ray.direction);
    const Vec2 projected_2 = model.project(ray.direction * 3.0);
    EXPECT_LT((pt - projected).norm(), EPS);
    EXPECT_LT((pt - projected_2).norm(), EPS);
  }
}

TEST(CameraModel, principal_point) {
  const ProjectionCoefficients coeffs{
      .fx = 1.0, .fy = 1.0, .cx = -1600 * 0.5, .cy = -1067 * 0.5};

  const CameraModel model(coeffs);

  const Vec3 pt(1600.0, 1067.0, 1.0);
  std::cout << model.project(pt).transpose() << std::endl;

  const Vec3 pt2(1600.0 / 2.0, 1067.0 / 2.0, 1.0);
  std::cout << model.project(pt2).transpose() << std::endl;
}
}  // namespace estimation