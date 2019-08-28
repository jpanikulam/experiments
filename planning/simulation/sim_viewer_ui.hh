#pragma once

#include "out.hh"
#include "util/optional.hh"

#include "planning/simulation/sim_viewer_command.hh"
#include "planning/simulation/sim_viewer_types.hh"

#include <memory>

namespace jcc {
namespace simulation {

struct ManifestEntry {
  std::string name;
  std::string filename;
  bool passing = false;
  bool allowed = false;
};

struct MainMenuState {
  float sim_speed = 1.0;
  bool snap_to_grid = true;

  char filename_entry_buffer[128] = "";

  bool manifest_stale = true;
  std::map<std::string, std::vector<ManifestEntry>> manifest_elements;

  bool new_file = false;
  std::string new_file_name = "";

  EditorCommand cmd_queue_update;
};

void create_main_menu(const EditorState& editor_state, Out<MainMenuState> menu_state);

struct TaskPopupState {
  bool is_new = true;
  int hovered = -1;
  jcc::Vec3 creation_world_point;
};

jcc::Optional<std::shared_ptr<Command>> create_task_popup(
    const EditorState& editor_state, Out<TaskPopupState> task_popup_state);

}  // namespace simulation
}  // namespace jcc