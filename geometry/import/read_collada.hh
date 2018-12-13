#pragma once

#include <map>
#include <string>
#include <vector>

#include "geometry/tri_mesh.hh"
#include "sophus.hh"

namespace geometry {
namespace import {

class ColladaModel {
 public:
  struct Edge {
    std::string child_name;
    SE3 child_from_parent;
  };

  ColladaModel(const std::string& path);

  ~ColladaModel() = default;

  const std::map<std::string, std::vector<Edge>>& adjacency() const {
    return adjacency_;
  }
  const std::map<std::string, TriMesh>& meshes() const {
    return meshes_;
  }

  const std::string& root() const {
    return root_;
  }

 private:
  void allocate(const std::string& path);

  std::string root_;

  std::map<std::string, std::vector<Edge>> adjacency_;
  std::map<std::string, TriMesh> meshes_;
};

}  // namespace import
}  // namespace geometry