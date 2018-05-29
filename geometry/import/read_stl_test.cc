#include "testing/gtest.hh"
#include "geometry/import/read_stl.hh"

namespace geometry {
namespace import {

TEST(ReadStlTest, read_stl) {
  const std::string file_path = "/home/jacob/repos/experiments/data/cube_shape.stl";
  read_stl(file_path);
}
}
}
