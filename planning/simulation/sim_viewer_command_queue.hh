#pragma once

#include <stack>

#include "out.hh"
#include "planning/simulation/sim_viewer_command.hh"
#include "planning/simulation/sim_viewer_types.hh"

namespace jcc {
namespace simulation {

class CommandQueue {
 public:
  void commit(const std::shared_ptr<Command> command, Out<EditorState> editor_state) {
    command->commit(editor_state);
    committed_.push_back(command);
    undone_commands_.clear();
  }

  bool can_undo() const {
    return !committed_.empty();
  }

  void undo(Out<EditorState> editor_state) {
    if (committed_.empty()) {
      return;
    }
    const auto command = committed_.back();
    command->undo(editor_state);
    undone_commands_.push_back(command);
    committed_.pop_back();
  }

  bool can_redo() const {
    return !undone_commands_.empty();
  }

  void redo(Out<EditorState> editor_state) {
    if (undone_commands_.empty()) {
      return;
    }

    const auto command = undone_commands_.back();
    command->commit(editor_state);
    committed_.push_back(command);
    undone_commands_.pop_back();
  }

  void save(const std::string& filepath);
  void load(const std::string& filepath, Out<EditorState> editor_state);

 private:
  std::vector<std::shared_ptr<Command>> committed_;
  std::vector<std::shared_ptr<Command>> undone_commands_;
};
}  // namespace simulation
}  // namespace jcc