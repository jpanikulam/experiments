/*******************************************************************************
 * Copyright (c) 2019 Main Street Autonomy LLC.
 ******************************************************************************/

#include "geometry/spatial/bounding_volume_hierarchy.hh"

#include "geometry/import/read_stl.hh"
#include "geometry/spatial/triangle_volume.hh"

#include "eigen.hh"
#include "testing/gtest.hh"

namespace geometry {
namespace spatial {

// Test intersection with a single triangle
TEST(TestBoundingVolumeHierarchy, easy_intersection) {
  //
  // Setup
  //

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(1);
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(1);

  const std::array<Eigen::Vector3d, 3> triangle_vertices = {
      {Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()}};
  triangles.emplace_back(triangle_vertices);
  tri_ptrs.push_back(&triangles.back());

  const Eigen::Vector3d ray_origin = Eigen::Vector3d(0.0, 0.5, -1.0);
  const Eigen::Vector3d ray_direction = Eigen::Vector3d::UnitZ();
  const Ray test_ray{ray_origin, ray_direction};

  geometry::spatial::BoundingVolumeHierarchy bvh;

  bvh.build(tri_ptrs);

  //
  // Test
  //

  const auto result = bvh.intersect(test_ray);

  //
  // Verification
  //

  EXPECT_TRUE(result.intersected);
  EXPECT_DOUBLE_EQ(result.distance, 1.0);
}

TEST(TestBoundingVolumeHierarchy, stacked_intersection) {
  //
  // Setup
  //

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(5);
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(5);

  for (int k = 0; k < 5; ++k) {
    const Eigen::Vector3d offset = Eigen::Vector3d::UnitZ() * k;
    const std::array<Eigen::Vector3d, 3> triangle_vertices = {
        {Eigen::Vector3d::UnitX() + offset, -Eigen::Vector3d::UnitX() + offset,
         Eigen::Vector3d::UnitY() + offset}};
    triangles.emplace_back(triangle_vertices);
  }

  for (size_t k = 0; k < triangles.size(); ++k) {
    tri_ptrs.push_back(&triangles[k]);
  }

  geometry::spatial::BoundingVolumeHierarchy bvh;
  bvh.build(tri_ptrs);

  const Eigen::Vector3d ray_origin = Eigen::Vector3d(0.0, 0.5, -1.0);
  const Eigen::Vector3d ray_direction = Eigen::Vector3d::UnitZ();
  const Ray test_ray{ray_origin, ray_direction};

  //
  // Test
  //

  const auto result = bvh.intersect(test_ray);

  //
  // Verification
  //

  EXPECT_TRUE(result.intersected);
  EXPECT_DOUBLE_EQ(result.distance, 1.0);
}

TEST(TestBoundingVolumeHierarchy, easy_nonintersection) {
  //
  // Setup
  //

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(1);
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(1);

  const std::array<Eigen::Vector3d, 3> triangle_vertices = {
      {Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()}};
  triangles.emplace_back(triangle_vertices);
  tri_ptrs.push_back(&triangles.back());

  geometry::spatial::BoundingVolumeHierarchy bvh;
  bvh.build(tri_ptrs);

  const Eigen::Vector3d ray_origin = Eigen::Vector3d(0.0, 0.5, -1.0);
  const Eigen::Vector3d ray_direction = -Eigen::Vector3d::UnitZ();
  const Ray test_ray{ray_origin, ray_direction};

  //
  // Test
  //

  const auto result = bvh.intersect(test_ray);

  //
  // Verification
  //

  EXPECT_FALSE(result.intersected);
}

TEST(TestBoundingVolumeHierarchy, stacked_nonintersection) {
  //
  // Setup
  //

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(5);
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(5);

  for (int k = 0; k < 5; ++k) {
    const Eigen::Vector3d offset = Eigen::Vector3d::UnitZ() * k;
    const std::array<Eigen::Vector3d, 3> triangle_vertices = {
        {Eigen::Vector3d::UnitX() + offset, -Eigen::Vector3d::UnitX() + offset,
         Eigen::Vector3d::UnitY() + offset}};
    triangles.emplace_back(triangle_vertices);
  }

  for (size_t k = 0; k < triangles.size(); ++k) {
    tri_ptrs.push_back(&triangles[k]);
  }

  geometry::spatial::BoundingVolumeHierarchy bvh;
  bvh.build(tri_ptrs);

  const Eigen::Vector3d ray_origin = Eigen::Vector3d(0.0, 0.5, -1.0);
  const Eigen::Vector3d ray_direction = -Eigen::Vector3d::UnitZ();
  const Ray test_ray{ray_origin, ray_direction};

  //
  // Test
  //

  const auto result = bvh.intersect(test_ray);

  //
  // Verification
  //

  EXPECT_TRUE(result.intersected);
  EXPECT_DOUBLE_EQ(result.distance, 1.0);
}

TEST(TestBoundingVolumeHierarchy, complex_intersection) {
  //
  // Setup
  //

  const std::string file_path = "test_stuff2.stl";
  const auto tri = *geometry::import::read_stl(file_path);

  std::vector<geometry::spatial::Volume *> tri_ptrs;
  tri_ptrs.reserve(tri.triangles.size());
  std::vector<geometry::spatial::TriangleVolume> triangles;
  triangles.reserve(tri.triangles.size());

  for (size_t k = 0; k < tri.triangles.size(); ++k) {
    triangles.emplace_back(tri.triangles[k].vertices);
    tri_ptrs.push_back(&triangles.back());
  }

  geometry::spatial::BoundingVolumeHierarchy bvh;
  bvh.build(tri_ptrs);

  const Eigen::Vector3d ray_origin = Eigen::Vector3d(0.0, 0.5, -1.0);
  const Eigen::Vector3d ray_direction = -Eigen::Vector3d::UnitZ();
  const Ray test_ray{ray_origin, ray_direction};

  //
  // Test
  //
  const auto result = bvh.intersect(test_ray);

  //
  // Verification
  //

  EXPECT_TRUE(result.intersected);
  EXPECT_DOUBLE_EQ(result.distance, 1.3);
}

}  // namespace spatial
}  // namespace geometry
