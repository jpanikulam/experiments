#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include <map>

namespace jcc {
namespace simulation {

enum class ElementType {
  NoGoZone = 0,  //
  GoZone = 1,    //
  LookZone = 2   //
};

struct Element {
  ElementType element_type;
  SE3 world_from_object;
  std::map<std::string, jcc::Vec3> properties;
};

struct EditorState {
  std::map<int, Element> elements;

  bool world_clicked = false;
  jcc::Vec3 world_click_pos;
};

}  // namespace simulation
}  // namespace jcc