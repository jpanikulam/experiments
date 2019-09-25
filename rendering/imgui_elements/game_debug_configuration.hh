#pragma once
#include <cstddef>
namespace jcc {
struct Debug {
  bool use_normals = false;
  bool use_rsm = false;
  bool wireframe = false;
  int polygon_mode = 1;
};
struct GameDebugConfiguration {
  Debug debug;
};
} // namespace jcc
