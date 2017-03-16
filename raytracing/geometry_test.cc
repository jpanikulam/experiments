// Tested af

#include "geometry.hh"

#include <gtest/gtest.h>

// todo
#include <type_traits>

namespace rt = raytrace;

using Vec2 = Eigen::Vector2d;

// Ray pointing at an obvious intersection point
TEST(Primitives, ray_line_intersection_nominal) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray ray(Vec2(0.5, 0.0), Vec2(1.0, 0.0));

  // Equivalent lines
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));
  const rt::Line line2(Vec2(1.0, 0.0), Vec2(0.0, -1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  Vec2 intersection2;
  bool intersected2 = rt::ray_line_intersection(ray, line2, out(intersection2));

  //
  // Verification
  //

  const Vec2 expected_intersection(1.0, 0.0);
  EXPECT_TRUE(intersected);
  EXPECT_TRUE(intersected2);

  EXPECT_NEAR(intersection(0), expected_intersection(0), EPS);
  EXPECT_NEAR(intersection(1), expected_intersection(1), EPS);

  EXPECT_NEAR(intersection(0), intersection2(0), EPS);
  EXPECT_NEAR(intersection(1), intersection2(1), EPS);
}

// The ray is pointing the wrong direction
TEST(Primitives, ray_line_intersection_backwards) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray  ray(Vec2(0.0, 0.0), Vec2(-1.0, 0.0));
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  //
  // Verification
  //

  const Vec2 expected_intersection(1.0, 0.0);

  // Intersection should have failed, but we still should have the place where the reversed ray
  // WOULD have intersected the line
  EXPECT_FALSE(intersected);
  EXPECT_NEAR(intersection(0), expected_intersection(0), EPS);
  EXPECT_NEAR(intersection(1), expected_intersection(1), EPS);
}

// The ray is parallel to the line
TEST(Primitives, ray_line_intersection_parallel) {
  //
  // Setup
  //

  const rt::Ray  ray(Vec2(0.0, 0.0), Vec2(0.0, 1.0));
  const rt::Line line(Vec2(1.0, 0.0), Vec2(0.0, 1.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_intersection(ray, line, out(intersection));

  //
  // Verification
  //

  // No intersection if the lines are parallel (Up to precision)
  EXPECT_FALSE(intersected);
}

// Ray pointed at line segment
TEST(Primitives, ray_line_segment_intersection) {
  constexpr double EPS = 1e-6;

  //
  // Setup
  //

  const rt::Ray ray(Vec2(0.0, 1.0), Vec2(0.0, -1.0));

  // Same line segment with switched ordering
  const rt::LineSegment segment(Vec2(1.0, 0.0), Vec2(-1.0, 0.0));
  const rt::LineSegment segment2(Vec2(-1.0, 0.0), Vec2(1.0, 0.0));

  //
  // Action
  //

  Vec2 intersection;
  bool intersected = rt::ray_line_segment_intersection(ray, segment, out(intersection));

  Vec2 intersection2;
  bool intersected2 = rt::ray_line_segment_intersection(ray, segment2, out(intersection2));

  //
  // Verification
  //

  EXPECT_TRUE(intersected);
  EXPECT_TRUE(intersected2);

  EXPECT_NEAR(intersection(0), intersection2(0), EPS);
  EXPECT_NEAR(intersection(1), intersection2(1), EPS);

  const Vec2 expected_intersection(0.0, 0.0);
  EXPECT_NEAR(intersection(0), expected_intersection(0), EPS);
  EXPECT_NEAR(intersection(1), expected_intersection(1), EPS);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}