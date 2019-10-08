#pragma once

#include "eigen.hh"

#include <string>

namespace viewer {

enum class SolarizedColor {
  Base03 = 0,
  Base02,
  Base01,
  Base00,
  Base0,
  Base1,
  Base2,
  Base3,
  Yellow,
  Orange,
  Red,
  Magenta,
  Violet,
  Blue,
  Cyan,
  Green,
  SIZE
};

jcc::Vec4 solarized_color(const std::string& name);
jcc::Vec4 solarized_color(const SolarizedColor id);

}  // namespace viewer