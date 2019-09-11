#include "rendering/assets/load_voxel_asset.hh"

#include "geometry/spatial/bounding_box.hh"
#include "logging/assert.hh"

#include <map>

namespace jcc {

namespace {

uint32_t lexicographic_index(int8_t x, int8_t y, int8_t z) {
  return (x << 16) | (y << 8) | z;
}

using Vec3i8 = Eigen::Matrix<int8_t, 3, 1>;

Vec3i8 deindex(uint32_t val) {
  Vec3i8 pos;
  pos[0] = (val >> 16) & 0xff;
  pos[1] = (val >> 8) & 0xff;
  pos[2] = val & 0xff;
  return pos;
}

struct TriangleFaceMesh {
  std::vector<Vec3ui32> faces;
  std::vector<jcc::Vec3f> vertices;
  std::vector<jcc::Vec3f> normals;
};

class SparseUniformByteGrid {
 public:
  using BBox = geometry::spatial::BoundingBox<3>;
  SparseUniformByteGrid(const geometry::VoxelList& voxel_list) : voxel_list_(voxel_list) {
    BBox bbox;
    const std::array<Vec3i8, 6> offsets = {Vec3i8(+1, 0, 0),  //
                                           Vec3i8(-1, 0, 0),  //
                                           Vec3i8(0, +1, 0),  //
                                           Vec3i8(0, -1, 0),  //
                                           Vec3i8(0, 0, +1),  //
                                           Vec3i8(0, 0, -1)};

    //
    // Build the sparse grid
    //
    for (std::size_t k = 0; k < voxel_list.voxels.size(); ++k) {
      const auto& voxel = voxel_list.voxels[k];
      const uint32_t table_entry_id = lexicographic_index(voxel.x, voxel.y, voxel.z);
      sparse_grid_[table_entry_id] = k;
      bbox.expand(jcc::Vec3(voxel.x, voxel.y, voxel.z));
    }

    std::cout << voxel_list.size.transpose() << std::endl;
    std::cout << "Estimated" << std::endl;
    std::cout << bbox.lower().transpose() << std::endl;
    std::cout << bbox.upper().transpose() << std::endl;

    //
    // Build the adjacency table
    //
    for (const auto& ab : sparse_grid_) {
      auto& local_adj = adjacency_[ab.first];

      const Vec3i8 pos = deindex(ab.first);

      for (int j = 0; j < 6; ++j) {
        const Vec3i8 explore = pos + offsets[j];
        const uint32_t id = lexicographic_index(explore.x(), explore.y(), explore.z());

        if (sparse_grid_.count(id) == 0u) {
          local_adj[j] = -1;
        } else {
          local_adj[j] = sparse_grid_[id];
        }
      }
    }
  }

  bool has(int8_t x, int8_t y, int8_t z) const {
    const uint32_t id = lexicographic_index(x, y, z);
    return 0u != sparse_grid_.count(id);
  }

  int get(int8_t x, int8_t y, int8_t z) const {
    const uint32_t id = lexicographic_index(x, y, z);
    return sparse_grid_.at(id);
  }

  TriangleFaceMesh mesh() const {
    const std::vector<jcc::Vec3f> cube_vertices = {
        jcc::Vec3f(-1.0, -1.0, 1.0),   //
        jcc::Vec3f(1.0, -1.0, 1.0),    //
        jcc::Vec3f(1.0, 1.0, 1.0),     //
        jcc::Vec3f(-1.0, 1.0, 1.0),    //
        jcc::Vec3f(-1.0, -1.0, -1.0),  //
        jcc::Vec3f(1.0, -1.0, -1.0),   //
        jcc::Vec3f(1.0, 1.0, -1.0),    //
        jcc::Vec3f(-1.0, 1.0, -1.0)    //
    };

    const std::vector<std::array<Vec3ui32, 2>> cube_faces = {
        // +X Face
        {Vec3ui32(1, 5, 6), Vec3ui32(6, 2, 1)},
        // -X Face
        {Vec3ui32(4, 0, 3), Vec3ui32(3, 7, 4)},
        // +Y Face
        {Vec3ui32(3, 2, 6), Vec3ui32(6, 7, 3)},
        // -Y Face
        {Vec3ui32(4, 5, 1), Vec3ui32(1, 0, 4)},
        // +Z Face
        {Vec3ui32(0, 1, 2), Vec3ui32(2, 3, 0)},
        // -Z Face
        {Vec3ui32(7, 6, 5), Vec3ui32(5, 4, 7)}};

    std::vector<Vec3ui32> faces;
    std::vector<jcc::Vec3f> vertices;
    std::vector<jcc::Vec3f> normals;
    constexpr double VX_SIZE = 0.1;

    for (const auto& vxl : voxel_list_.voxels) {
      const uint32_t id = lexicographic_index(vxl.x, vxl.y, vxl.z);
      const auto& adj = adjacency_.at(id);
      const jcc::Vec3f offset = jcc::Vec3i(vxl.x, vxl.y, vxl.z).cast<float>();

      for (std::size_t face_ind = 0; face_ind < adj.size(); ++face_ind) {
        const int neighbor_id = adj[face_ind];
        // Only apply the face if there is no neighbor in this direction
        if (neighbor_id == -1) {
          for (const auto& face : cube_faces[face_ind]) {
            faces.push_back(
                jcc::Vec3ui32(vertices.size(), vertices.size() + 1, vertices.size() + 2));

            const jcc::Vec3f ab = cube_vertices[face[2]] - cube_vertices[face[0]];
            const jcc::Vec3f ca = cube_vertices[face[1]] - cube_vertices[face[0]];
            const jcc::Vec3f normal = ab.cross(ca).normalized();

            for (int vx_ind = 0; vx_ind < 3; ++vx_ind) {
              const jcc::Vec3f vertex = cube_vertices[face[vx_ind]];
              const jcc::Vec3f vertex_offset = VX_SIZE * ((0.5f * vertex) + offset);
              vertices.push_back(vertex_offset);
              normals.push_back(normal);
            }
          }
        }
      }
    }
    return TriangleFaceMesh{faces, vertices, normals};
  }

 private:
  geometry::VoxelList voxel_list_;
  std::map<uint32_t, int> sparse_grid_;

  // The adjacency can be stored as a bitfield instead of an index, but the bitfield
  // approach requires more map lookups
  std::map<uint32_t, std::array<int, 6>> adjacency_;
};

}  // namespace

VoxelAsset::VoxelAsset(const geometry::VoxelList& voxel_list) : voxel_list_(voxel_list) {
  vertices_.clear();
  faces_.clear();
  normals_.clear();

  const SparseUniformByteGrid sug(voxel_list);
  const auto msh = sug.mesh();
  vertices_ = msh.vertices;
  normals_ = msh.normals;
  faces_ = msh.faces;
  JASSERT_EQ(normals_.size(), vertices_.size(),
             "Different number of normals and vertices");
}

VoxelAsset load_voxel_asset(const std::string& path) {
  const auto vxlist = geometry::read_vox(path);
  JASSERT(static_cast<bool>(vxlist), "Could not load voxel list");
  return VoxelAsset(*vxlist);
}

}  // namespace jcc