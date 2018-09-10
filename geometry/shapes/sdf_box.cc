#include "geometry/shapes/sdf_box.hh"

#include "geometry/spatial/fit_bounding_box.hh"
#include "geometry/spatial/nearest_triangle.hh"


namespace geometry {
namespace shapes {

using Vec3 = Eigen::Vector3d;

Vec3 SampledSdf::position_for_voxel_index(int i) const {
  const int& nx = voxels_per_axis_[0];
  const int& ny = voxels_per_axis_[1];

  const Vec3 p(i % nx, (i / nx) % ny, i / (nx * ny));
  // const Vec3 center =
      // bbox_.center() + (Vec3(voxel_size_m_, voxel_size_m_, voxel_size_m_) * 0.5);
  // const Vec3 center = bbox_.center();

  return (p * voxel_size_m_) + bbox_.lower();
}

void SampledSdf::populate_voxel_grid(const TriMesh& mesh,
                                     Out<std::vector<double>> distances,
                                     Out<std::vector<int>> indices) const {
  // No more than 1 million voxels (4mb)
  assert(voxel_size_m_ > 0.0);
  constexpr int MAX_SIZE = 1e6;

  const int total_voxels =
      voxels_per_axis_[0] * voxels_per_axis_[1] * voxels_per_axis_[2];
  assert(total_voxels < MAX_SIZE);

  distances->resize(total_voxels);
  indices->resize(total_voxels);

  for (int i = 0; i < total_voxels; ++i) {
    const Vec3 voxel_center = position_for_voxel_index(i);
    const auto tri = spatial::find_nearest_triangle(voxel_center, mesh);
    (*distances)[i] = tri.distance;
    (*indices)[i] = tri.simplex_index;
  }
}

SampledSdf::SampledSdf(const TriMesh& mesh, double meters_per_voxel) {
  voxel_size_m_ = meters_per_voxel;
  {
    bbox_ = spatial::fit_bounding_box(mesh);
    const Vec3 bbox_size_m = bbox_.upper() - bbox_.lower();
    const Vec3 num_voxels_per_axis = bbox_size_m / meters_per_voxel;
    voxels_per_axis_ = num_voxels_per_axis.array().ceil().cast<int>();
  }

  populate_voxel_grid(mesh, out(signed_distances_), out(indices_));
}

}  // namespace shapes
}  // namespace geometry