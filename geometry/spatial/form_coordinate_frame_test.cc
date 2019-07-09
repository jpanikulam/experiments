
#include "testing/gtest.hh"

#include "eigen.hh"
#include "geometry/spatial/form_coordinate_frame.hh"

namespace geometry {
namespace spatial {

TEST(TestFormCoordinateFrame, frame_test) {
  const jcc::Vec3 desired_x_world(1.0, 1.0, 0.0);
  const Unit3 zhat_world(jcc::Vec3(0.0, 0.0, 1.0));
  const SO3 world_from_frame = form_coordinate_frame_from_zhat_and_x(
      Unit3(jcc::Vec3(0.0, 0.0, 1.0)), desired_x_world);

  const Unit3 desired_x_frame = world_from_frame.inverse() * Unit3(desired_x_world);
  EXPECT_DOUBLE_EQ(desired_x_frame.dot(jcc::Vec3::UnitX()), 1.0);

  const Unit3 desired_z_frame = world_from_frame.inverse() * zhat_world;
  EXPECT_DOUBLE_EQ(desired_z_frame.dot(jcc::Vec3::UnitZ()), 1.0);
}

}  // namespace spatial
}  // namespace geometry
