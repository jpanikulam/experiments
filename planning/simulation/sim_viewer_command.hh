#pragma once

#include "eigen.hh"
#include "out.hh"

#include "planning/simulation/sim_viewer_types.hh"

namespace jcc {
namespace simulation {

struct Command {
  virtual ~Command() = default;
  virtual void commit(Out<EditorState> state) = 0;
  virtual void undo(Out<EditorState> state) = 0;

  std::string desc;
  virtual std::string description() {
    return desc;
  };
};

struct CreateElementCommand : Command {
  int id;
  ElementType element_type;
  SE3 world_from_object;
  std::map<std::string, jcc::Vec3> initial_properties;

  std::string desc;

  void commit(Out<EditorState> state) {
    Element obj;
    obj.element_type = element_type;
    obj.world_from_object = world_from_object;
    obj.properties = initial_properties;
    state->elements[id] = obj;
  }

  void undo(Out<EditorState> state) {
  state->elements.erase(id);
  }
};

// TODO: Generate these
//
// - extents : vector
// - success time: scalar
struct ModifyElementCommand : Command {
  int id;
  std::string property;
  jcc::Vec3 value;
  std::string desc;

  bool had_previous_value = false;
  jcc::Vec3 previous_value;

  void commit(Out<EditorState> state) {
    if (state->elements[id].properties.count(property)) {
      previous_value = state->elements[id].properties[property];
      had_previous_value = true;
    }
    state->elements[id].properties[property] = value;
  }

  void undo(Out<EditorState> state) {
    if (had_previous_value) {
      state->elements[id].properties[property] = previous_value;
    } else {
      state->elements[id].properties.erase(property);
    }
  }
};

}  // namespace simulation
}  // namespace jcc