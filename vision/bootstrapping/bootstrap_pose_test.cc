#include "bootstrap_pose.hh"

#include "testing/gtest.hh"

namespace slam {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

class BootstrapPoseTest : public ::testing::Test {
 protected:
  BootstrapPoseTest() {
    constexpr double FX = 10.0;
    constexpr double FY = 10.0;
    constexpr double CX = 5.0;
    constexpr double CY = 5.0;
    cam_model_          = CameraModel(FX, FY, CX, CY);
  }

  std::vector<Vec3> generate_object(const int num_pts) {
    std::vector<Vec3> points_object_frame(num_pts);

    for (int k = 0; k < num_pts; ++k) {
      const Vec3 pt_object_frame = Vec3::Random();
      points_object_frame[k]     = pt_object_frame;
    }
    return points_object_frame;
  }

  std::vector<Vec2> view_object_from_pose(const std::vector<Vec3>& points_object_frame, const SE3& camera_from_object) {
    std::vector<Vec2> projected_points(points_object_frame.size());
    for (size_t k = 0; k < points_object_frame.size(); ++k) {
      projected_points[k] = cam_model_.project(camera_from_object * points_object_frame[k]);
    }
    return projected_points;
  }

  CameraModel cam_model_ = CameraModel(Eigen::Matrix3d::Identity());
};

TEST(BootstrapPoseTest, can_bootstrap) {
  //
  // Setup
  //

  const Vec3 vv = Vec3::UnitX();
  const SO3  R  = SO3::exp(Vec3(0.0, 0.0, 0.0));

  std::cout << (R * vv).transpose() << std::endl;
}
}