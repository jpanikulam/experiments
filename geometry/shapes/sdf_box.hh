#pragma once


#include "geometry/spatial/bounding_box.hh"
#include "geometry/tri_mesh.hh"

#include "out.hh"
#include "eigen.hh"

namespace geometry {
namespace shapes {

class SampledSdf {
public:
  SampledSdf(const TriMesh& mesh, double voxel_size_m = 0.1);

  double signed_distance();

  // Detail!
  const std::vector<double>& get_signed_distances() const {
    return signed_distances_;
  }

  Eigen::Vector3d position_for_voxel_index(int i) const;

  const spatial::BoundingBox<3>& bounding_box() const {
    return bbox_;
  }


 private:
  void populate_voxel_grid(const TriMesh& mesh,
                           Out<std::vector<double>> distances,
                           Out<std::vector<int>> indices) const;

  // The bounding box for this voxel grid
  spatial::BoundingBox<3> bbox_;

  // Signed distance to each vertex
  std::vector<double> signed_distances_;

  // Indices of the nearest triangle to each vertex
  std::vector<int> indices_;

  Eigen::Matrix<int, 3, 1> voxels_per_axis_;
  double voxel_size_m_ = -1.0;
};

}  // namespace shapes
}  // namespace geometry