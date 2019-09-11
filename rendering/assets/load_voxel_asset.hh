#pragma once

#include "eigen.hh"
#include "geometry/import/read_vox.hh"

#include <string>
#include <vector>

namespace jcc {
using Vec3ui32 = Eigen::Matrix<uint32_t, 3, 1>;

class VoxelAsset {
 public:
  VoxelAsset() = default;
  VoxelAsset(const geometry::VoxelList& voxel_list);

  const std::vector<jcc::Vec3f>& vertices() const {
    return vertices_;
  };

  const std::vector<Vec3ui32>& faces() const {
    return faces_;
  };

  const std::vector<jcc::Vec3f>& normals() const {
    return normals_;
  };

 private:
  geometry::VoxelList voxel_list_;

  std::vector<jcc::Vec3f> vertices_;
  std::vector<jcc::Vec3f> normals_;
  std::vector<Vec3ui32> faces_;
};

VoxelAsset load_voxel_asset(const std::string& path);

}  // namespace jcc