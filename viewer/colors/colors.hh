#pragma once

#include "eigen.hh"

namespace viewer {
// TODO: Add a color scheme

inline jcc::Vec4 black() {
  return jcc::Vec4(0.0, 0.0, 0.0, 1.0);
}

inline jcc::Vec4 white() {
  return jcc::Vec4(1.0, 1.0, 1.0, 1.0);
}

inline jcc::Vec4 red() {
  return jcc::Vec4(1.0, 0.0, 0.0, 1.0);
}

inline jcc::Vec4 lime() {
  return jcc::Vec4(0.0, 1.0, 0.0, 1.0);
}

inline jcc::Vec4 blue() {
  return jcc::Vec4(0.0, 0.0, 1.0, 1.0);
}

inline jcc::Vec4 yellow() {
  return jcc::Vec4(1.0, 1.0, 0.0, 1.0);
}

inline jcc::Vec4 cyan() {
  return jcc::Vec4(0.0, 1.0, 1.0, 1.0);
}

inline jcc::Vec4 magenta() {
  return jcc::Vec4(1.0, 0.0, 1.0, 1.0);
}

inline jcc::Vec4 silver() {
  return jcc::Vec4(0.75, 0.75, 0.75, 1.0);
}

inline jcc::Vec4 gray() {
  return jcc::Vec4(0.5, 0.5, 0.5, 1.0);
}

inline jcc::Vec4 maroon() {
  return jcc::Vec4(0.5, 0.0, 0.0, 1.0);
}

inline jcc::Vec4 olive() {
  return jcc::Vec4(0.5, 0.5, 0.0, 1.0);
}

inline jcc::Vec4 green() {
  return jcc::Vec4(0.0, 0.5, 0.0, 1.0);
}

inline jcc::Vec4 purple() {
  return jcc::Vec4(0.5, 0.0, 0.5, 1.0);
}

inline jcc::Vec4 teal() {
  return jcc::Vec4(0.0, 0.5, 0.5, 1.0);
}

inline jcc::Vec4 navy() {
  return jcc::Vec4(0.0, 0.0, 0.5, 1.0);
}

}  // namespace viewer