#pragma once

#include "eigen.hh"

#include <string>
#include <vector>

namespace geometry {
namespace import {
struct Triangle {
  std::array<Eigen::Vector3d, 3> vertices;
  Eigen::Vector3d normal;
};

struct TriMesh {
  std::vector<Triangle> triangles;
};

TriMesh read_stl(const std::string &file_path);
}
}
