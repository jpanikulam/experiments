#pragma once

#include "eigen.hh"
#include "sophus.hh"

#include <map>

namespace jcc {
namespace simulation {
//
// Concept: This is a static description of the scene and should only be edited through
// undoable commands
//

struct Robot {
  SE3 world_from_robot;
  std::string model;

  std::map<std::string, jcc::Vec3> properties;
};

enum class SelectionState {
  None = 0,     //
  Hovered = 1,  //
  Selected = 2  //
};

enum class ElementType {
  INVALID = 0,   //
  NoGoZone = 1,  //
  GoZone = 2,    //
  LookZone = 3,  //
  Object = 4     //
};

enum class EditorCommand {
  None = 0,  //
  Undo = 1,  //
  Redo = 2,  //
  Save = 3,  //
  Open = 4   //
};

struct Element {
  ElementType element_type;
  SE3 world_from_element;
  std::string model;

  std::map<std::string, jcc::Vec3> properties;

  SelectionState selection_state = SelectionState::None;
};

struct EditorConfig {
  // Consider: Allow UI elements to edit this directly, so there is a single source of
  // truth Otherwise: Code-generate some kind of thing
  double sim_speed = 0.05;
  bool snap_to_grid = true;
};

struct EditorState {
  EditorConfig config;
  std::string filename;

  int time_scrub = 0;

  // TODO: Rename to element_from_id
  std::map<int, Element> elements;
  Robot robot;

  //
  // World interaction
  //
  bool world_hovered = false;
  jcc::Vec3 world_click_pos;

  int hovered = -1;

  //
  // UI Affordances
  //
  bool can_undo = false;
  bool can_redo = false;
  bool any_robot_placed = false;
};

}  // namespace simulation
}  // namespace jcc