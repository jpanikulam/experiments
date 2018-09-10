#pragma once

#include "eigen.hh"

#include "geometry/tri_mesh.hh"

#include <string>
#include <vector>

namespace geometry {
namespace import {

TriMesh read_stl(const std::string &file_path);
}
}
