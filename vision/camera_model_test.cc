#include "camera_model.hh"

#include "testing/gtest.hh"

namespace slam {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

TEST(CameraModel, project_inverts_unproject) {
  //
  // Setup
  //

  constexpr double  FX = 999.2;
  constexpr double  FY = 1000.1;
  constexpr double  CX = 0.0;
  constexpr double  CY = 0.0;
  const CameraModel model(FX, FY, CX, CY);

  //
  // Action / Verification
  //

  constexpr double EPS = 1e-6;
  {
    const Vec2 pt(630, 290);
    const Vec3 ray       = model.unproject(pt);
    const Vec2 projected = model.project(ray);

    EXPECT_LT((pt - projected).norm(), EPS);
  }

  {
    const Vec2 pt(12, 82);
    const Vec3 ray       = model.unproject(pt);
    const Vec2 projected = model.project(ray);
    EXPECT_LT((pt - projected).norm(), EPS);
  }
}

TEST(CameraModel, principal_point) {
  constexpr double  FX = 1.0;
  constexpr double  FY = 1.0;
  constexpr double  CX = -1600 * 0.5;
  constexpr double  CY = -1067 * 0.5;
  const CameraModel model(FX, FY, CX, CY);

  const Vec3 pt(1600.0, 1067.0, 1.0);
  std::cout << model.project(pt).transpose() << std::endl;

  const Vec3 pt2(1600.0 / 2.0, 1067.0 / 2.0, 1.0);
  std::cout << model.project(pt2).transpose() << std::endl;
}
}