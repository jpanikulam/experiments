#include "planning/simulation/sim_viewer_command_queue.hh"

#include <fstream>

namespace jcc {
namespace simulation {
void CommandQueue::save(const std::string& filepath) {
  std::ofstream myfile(filepath);
  YAML::Node cmd_queue;

  cmd_queue["committed"] = save_commands(committed_);
  cmd_queue["undone"] = save_commands(undone_commands_);

  myfile << cmd_queue;
}
void CommandQueue::load(const std::string& filepath, Out<EditorState> editor_state) {
  while (can_undo()) {
    // Destroy all commands
    undo(editor_state);
  }

  const YAML::Node node = YAML::LoadFile(filepath);
  const auto commit_commands = load_commands(node["committed"]);
  for (const auto& command : commit_commands) {
    commit(command, editor_state);
  }

  const auto undo_commands = load_commands(node["undone"]);
  for (const auto& command : undo_commands) {
    undone_commands_.push_back(command);
  }
}

}  // namespace simulation
}  // namespace jcc
