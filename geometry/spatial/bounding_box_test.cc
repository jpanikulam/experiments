
#include "testing/gtest.hh"

#include "eigen.hh"
#include "geometry/spatial/bounding_box.hh"

using Vec3 = Eigen::Vector3d;

namespace geometry {
namespace spatial {

TEST(TestBoundingBox, intersection_test) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {  // Translated beneath on y, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(0.0, -0.5, 0.5);
    ray.direction = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // Translated above on y, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(0.0, 1.5, 0.5);
    ray.direction = Vec3(0.0, -1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on z, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(0.5, 0.0, 1.5);
    ray.direction = Vec3(0.0, 0.0, -1.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on z, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(0.5, 0.0, -0.5);
    ray.direction = Vec3(0.0, 0.0, 1.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on x, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(-0.5, 0.0, 0.5);
    ray.direction = Vec3(1.0, 0.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {  // translated above on x, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(1.5, 0.0, 0.5);
    ray.direction = Vec3(-1.0, 0.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }

  {
    geometry::Ray ray;
    ray.origin = Vec3(0.0, -0.5, 0.5);
    ray.direction = Vec3(0.0, -1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }

  {
    geometry::Ray ray;
    ray.origin = Vec3(0.0, -1.0, 0.5);
    ray.direction = Vec3(0.0, -1.0, -1.0).normalized();
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }
}

TEST(TestBoundingBox, contains_ray) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {  // Translated beneath on y, pointing at box
    geometry::Ray ray;
    ray.origin = Vec3(0.5, 0.5, 0.5);
    ray.direction = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    EXPECT_TRUE(result.intersected);

    constexpr double EPS = 1e-6;
    ASSERT_TRUE(bbox.contains(ray.origin));
    EXPECT_NEAR(result.distance, 0.5, EPS);
  }
}

TEST(TestBoundingBox, does_not_intersect) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.0, 0.0, 0.0));
  bbox.expand(Vec3(1.0, 1.0, 1.0));

  {
    geometry::Ray ray;
    ray.origin = Vec3(1.5, 1.5, 1.5);
    ray.direction = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }

  {
    geometry::Ray ray;
    ray.origin = Vec3(0.0, 0.0, 1.5);
    ray.direction = Vec3(0.0, 1.0, 0.0);
    const auto result = bbox.intersect(ray);

    ASSERT_FALSE(bbox.contains(ray.origin));
    EXPECT_FALSE(result.intersected);
  }
}

// Test that we fixed a bug found in testing the bounding volume hierarchy
// (Turns out, the slab method as described fails when parallel to the box)
TEST(TestBoundingBox, apparent_bug) {
  BoundingBox<3> bbox;
  bbox.expand(Vec3(0.534403, -0.364025, 8.25298));
  bbox.expand(Vec3(-2.01473, -3.04393, 2.45791));

  geometry::Ray ray;
  ray.origin = Vec3(0.0, 0.0, 0.75);
  ray.direction = Vec3(-1.0, -1.0, 0.0).normalized();
  const Intersection intersection = bbox.intersect(ray);

  EXPECT_FALSE(intersection.intersected);
}

TEST(TestBoundingBox, contains_aabb) {
  BoundingBox<3> bbox1;
  bbox1.expand(Vec3(0.0, 0.0, 0.0));
  bbox1.expand(Vec3(1.0, 1.0, 1.0));

  BoundingBox<3> bbox2;
  bbox2.expand(Vec3(0.1, 0.1, 0.1));
  bbox2.expand(Vec3(0.9, 0.9, 0.9));

  // Contains itself
  EXPECT_TRUE(bbox1.contains(bbox1));

  // Contains sub-box
  EXPECT_TRUE(bbox1.contains(bbox2));

  // Does not contain thing that it contains
  EXPECT_FALSE(bbox2.contains(bbox1));
}

TEST(TestBoundingBox, intersects_aabb) {
  BoundingBox<3> bbox1;
  bbox1.expand(Vec3(0.0, 0.0, 0.0));
  bbox1.expand(Vec3(1.0, 1.0, 1.0));

  BoundingBox<3> bbox2;
  bbox2.expand(Vec3(0.1, 0.1, 0.1));
  bbox2.expand(Vec3(0.9, 0.9, 0.9));

  BoundingBox<3> bbox3;
  bbox3.expand(Vec3(0.5, 0.5, 0.5));
  bbox3.expand(Vec3(1.5, 1.5, 1.5));

  BoundingBox<3> bbox4;
  bbox4.expand(Vec3(1.2, 1.2, 1.2));
  bbox4.expand(Vec3(1.5, 1.5, 1.5));

  // Intersects itself
  EXPECT_TRUE(bbox1.intersects(bbox1));

  // Intersects sub-box
  EXPECT_TRUE(bbox1.intersects(bbox2));

  // Intersects containing box
  EXPECT_TRUE(bbox2.intersects(bbox1));

  // Intersects overlapping box
  EXPECT_TRUE(bbox1.intersects(bbox3));

  // Does not intersect non-overlapping box
  EXPECT_FALSE(bbox1.intersects(bbox4));
}

}  // namespace spatial
}  // namespace geometry
