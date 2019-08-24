#pragma once

#include "out.hh"
#include "util/optional.hh"

#include "planning/simulation/sim_viewer_command.hh"
#include "planning/simulation/sim_viewer_types.hh"

#include <memory>

namespace jcc {
namespace simulation {

struct MainMenuState {
  float sim_speed = 1.0;
};

void create_main_menu(Out<MainMenuState> menu_state);

struct TaskPopupState {
  bool is_new = true;
  jcc::Vec3 creation_world_point;
};

jcc::Optional<std::shared_ptr<Command>> create_task_popup(
    const EditorState& editor_state, Out<TaskPopupState> task_popup_state);

}  // namespace simulation
}  // namespace jcc