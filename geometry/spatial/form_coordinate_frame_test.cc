
#include "testing/gtest.hh"

#include "eigen.hh"
#include "geometry/spatial/form_coordinate_frame.hh"

namespace geometry {
namespace spatial {

TEST(TestFormCoordinateFrame, frame_test) {
  const jcc::Vec3 desired_x_world(1.0, 1.0, 0.0);
  const SO3 world_from_frame = form_coordinate_frame_from_zhat_and_x(
      Unit3(jcc::Vec3(0.0, 0.0, 1.0)), desired_x_world);

  const jcc::Vec3 desired_x_frame = world_from_frame.inverse() * desired_x_world;
  EXPECT_GT(desired_x_frame.dot(jcc::Vec3::UnitX()), 0.99999);
}
}  // namespace spatial
}  // namespace geometry
