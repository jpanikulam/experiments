#pragma once

#include "viewer/gl_size.hh"
#include "viewer/gl_types.hh"

namespace viewer {

struct GlViewport {
  GlSize size;
  WindowPoint location;
};

}  // namespace viewer
