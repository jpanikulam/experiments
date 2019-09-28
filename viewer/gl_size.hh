#pragma once

namespace viewer {

struct GlSize {
  GlSize() = default;
  GlSize(int w, int h) {
    width  = w;
    height = h;
  }

  int width  = 0;
  int height = 0;
};

}  // viewer
