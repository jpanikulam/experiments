#pragma once
#include <cstddef>
namespace jcc {
struct Debug {
  bool wireframe = false;
  double theta = 3.14;
  double d = 0.438;
};
struct Shading {
  bool enable_shadows = true;
  bool misc_debug = false;
  bool srgb = false;
  bool use_rsm = true;
  bool show_light_probes = false;
};
struct GameDebugConfiguration {
  Debug debug;
  Shading shading;
};
} // namespace jcc
