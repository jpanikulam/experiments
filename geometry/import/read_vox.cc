#include "geometry/import/read_vox.hh"

#include "geometry/import/read_from_file.hh"
#include "logging/assert.hh"

#include <fstream>
#include <string>

namespace geometry {

struct ChunkDescription {
  std::string chunk_id;
  uint32_t n_bytes_chunk;
  uint32_t n_bytes_children;
};

// 2. Chunk Structure
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 1x4      | char       | chunk id
// 4        | int        | num bytes of chunk content (N)
// 4        | int        | num bytes of children chunks (M)

// N        |            | chunk content

// M        |            | children chunks
// -------------------------------------------------------------------------------
ChunkDescription read_chunk_header(std::ifstream& vox_file) {
  ChunkDescription desc;
  desc.chunk_id = read_string_from_file<4>(vox_file);
  desc.n_bytes_chunk = read_from_file<uint32_t>(vox_file);
  desc.n_bytes_children = read_from_file<uint32_t>(vox_file);
  return desc;
}

// 5. Chunk id 'SIZE' : model size
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | size x
// 4        | int        | size y
// 4        | int        | size z : gravity direction
// -------------------------------------------------------------------------------
std::array<uint32_t, 3> read_size_chunk(std::ifstream& vox_file) {
  std::array<uint32_t, 3> size;
  size[0] = read_from_file<uint32_t>(vox_file);
  size[1] = read_from_file<uint32_t>(vox_file);
  size[2] = read_from_file<uint32_t>(vox_file);
  return size;
}

// 6. Chunk id 'XYZI' : model voxels
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | numVoxels (N)
// 4 x N    | int        | (x, y, z, colorIndex) : 1 byte for each component
std::vector<Voxel> read_xyzi_chunk(std::ifstream& vox_file) {
  const uint32_t n_voxels = read_from_file<uint32_t>(vox_file);

  std::vector<Voxel> voxels;
  voxels.reserve(n_voxels);

  // Read all of the voxels
  for (uint32_t k = 0; k < n_voxels; ++k) {
    Voxel vxl;
    // TODO: Is this signed??
    vxl.x = read_from_file<int8_t>(vox_file);
    vxl.y = read_from_file<int8_t>(vox_file);
    vxl.z = read_from_file<int8_t>(vox_file);
    vxl.color_ind = read_from_file<int8_t>(vox_file);
    voxels.push_back(vxl);
  }

  return voxels;
}

//
// 7. Chunk id 'RGBA' : palette
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4 x 256  | int        | (R, G, B, A) : 1 byte for each component
//                       | * <NOTICE>
//                       | * color [0-254] are mapped to palette index [1-255], e.g :
//                       |
//                       | for ( int i = 0; i <= 254; i++ ) {
//                       |     palette[i + 1] = ReadRGBA();
//                       | }
// -------------------------------------------------------------------------------
//
std::array<uint32_t, 256> read_rba_chunk(std::ifstream& vox_file) {
  const auto rs = read_array_from_file<uint32_t, 256>(vox_file);

  return rs;
}

jcc::Optional<VoxelList> read_vox(const std::string& file_path) {
  std::ifstream vox_file(file_path, std::ios::binary);
  if (!vox_file.is_open()) {
    return {};
  }

  constexpr bool READ_HEADER = true;
  // Read the header
  if (READ_HEADER) {
    constexpr int HEADER_SIZE = 4;
    const std::string header_content = read_string_from_file<HEADER_SIZE>(vox_file);
    JASSERT_EQ(header_content, std::string("VOX "), "Vox file must start with 'VOX '");

    const uint32_t version = read_from_file<uint32_t>(vox_file);
    JASSERT_GE(version, 150u, "Version must be greater than 150");
  }

  const std::string chunk_id = read_string_from_file<4>(vox_file);
  JASSERT_EQ(chunk_id, "MAIN", "First chunk must be MAIN -- PACK is unsupported");
  const auto n_bytes_chunk = read_from_file<uint32_t>(vox_file);
  (void)n_bytes_chunk;
  const auto n_bytes_children = read_from_file<uint32_t>(vox_file);
  (void)n_bytes_children;

  VoxelList vxl_list;

  // TODO: Set default palette
  bool had_rgba = false;
  for (int k = 0; k < 3; ++k) {
    // const auto next_chunk = read_string_from_file<4>(vox_file);
    const auto next_chunk = read_chunk_header(vox_file);
    if (next_chunk.chunk_id == "SIZE") {
      const auto sz = read_size_chunk(vox_file);
      vxl_list.size = jcc::Vec3i(sz[0], sz[1], sz[2]);
    } else if (next_chunk.chunk_id == "XYZI") {
      vxl_list.voxels = read_xyzi_chunk(vox_file);
    } else if (next_chunk.chunk_id == "RGBA") {
      vxl_list.palette = read_rba_chunk(vox_file);
      had_rgba = true;
    }
  }
  JASSERT(had_rgba, "Default palette isn't supported, ask Jake, it'll take 30sec to add");

  return vxl_list;
}  // namespace geometry

jcc::Vec4 color_from_id(const VoxelList& voxel_list, const uint8_t id) {
  const uint64_t palette_val = voxel_list.palette[id - 1];

  jcc::Vec4 color;

  color[0] = static_cast<double>(0xff & (palette_val >> (8 * 0))) / 255.0;
  color[1] = static_cast<double>(0xff & (palette_val >> (8 * 1))) / 255.0;
  color[2] = static_cast<double>(0xff & (palette_val >> (8 * 2))) / 255.0;
  color[3] = static_cast<double>(0xff & (palette_val >> (8 * 3))) / 255.0;

  return color;
}

}  // namespace geometry