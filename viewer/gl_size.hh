#pragma once

namespace gl_viewer {

struct GlSize {
  GlSize() {
  }
  GlSize(int h, int w) {
    height = h;
    width  = w;
  }

  int height = 640;
  int width  = 480;
};

}  // gl_viewer
