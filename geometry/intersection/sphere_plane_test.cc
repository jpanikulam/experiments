#include "testing/gtest.hh"

#include "geometry/intersection/sphere_plane.hh"

namespace geometry {
namespace intersection {

using Vec3 = Eigen::Vector3d;

TEST(SpherePlane, plane_contains_center) {
  //
  // Setup
  //

  constexpr double SPHERE_RADIUS = 5.0;
  const shapes::Plane plane{Vec3::UnitX(), 0.0};
  const shapes::Sphere sphere{Vec3::Zero(), SPHERE_RADIUS};

  //
  // Action
  //

  const auto result = sphere_plane_intersection(sphere, plane);

  //
  // Verification
  //

  ASSERT_TRUE(bool(result));
  EXPECT_FLOAT_EQ(result->radius, SPHERE_RADIUS);

  // These should be *identical*
  EXPECT_EQ((result->plane.u_normal - plane.u_normal).squaredNorm(), 0.0);
  EXPECT_EQ(result->plane.distance_from_origin, plane.distance_from_origin);
}

TEST(SpherePlane, does_not_intersect) {
  //
  // Setup
  //

  constexpr double SPHERE_RADIUS = 5.0;
  const shapes::Plane plane{Vec3::UnitX(), 0.0};
  const shapes::Sphere sphere{Vec3::UnitX() * 10.0, SPHERE_RADIUS};

  //
  // Action
  //

  const auto result = sphere_plane_intersection(sphere, plane);

  //
  // Verification
  //

  EXPECT_FALSE(result);
}

TEST(SpherePlane, plane_does_not_contain_center) {
  //
  // Setup
  //

  constexpr double SPHERE_RADIUS = 1.0;
  const shapes::Plane plane{Vec3::UnitX(), 0.0};
  const shapes::Sphere sphere{Vec3::UnitX(), SPHERE_RADIUS};

  //
  // Action
  //

  const auto result = sphere_plane_intersection(sphere, plane);

  //
  // Verification
  //

  ASSERT_TRUE(bool(result));
  EXPECT_FLOAT_EQ(result->radius, 0.0);

  // These should be *identical*
  EXPECT_EQ((result->plane.u_normal - plane.u_normal).squaredNorm(), 0.0);
  EXPECT_EQ(result->plane.distance_from_origin, plane.distance_from_origin);
}

}  // namespace intersection
}  // namespace geometry