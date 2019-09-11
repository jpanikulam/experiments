#pragma once

#include "eigen.hh"
#include "util/optional.hh"

#include <string>
#include <vector>

namespace geometry {

struct Voxel {
  int8_t x;
  int8_t y;
  int8_t z;
  int8_t color_ind = -1;
};

struct VoxelList {
  jcc::Vec3i size;
  std::vector<Voxel> voxels;
};

// Read a .vox file
jcc::Optional<VoxelList> read_vox(const std::string& file_path);

jcc::Vec4 color_from_default_id(const uint8_t id);

}  // namespace geometry