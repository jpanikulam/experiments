#pragma once

namespace gl_viewer {

struct GlSize {
  GlSize() {
  }
  GlSize(int w, int h) {
    width  = w;
    height = h;
  }

  int width  = 0;
  int height = 0;
};

}  // gl_viewer
