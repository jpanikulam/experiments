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
    committed_.push(command);

    while (!undone_commands_.empty()) {
      undone_commands_.pop();
    }
  }

  bool can_undo() const {
    return !committed_.empty();
  }

  void undo(Out<EditorState> editor_state) {
    if (committed_.empty()) {
      return;
    }
    const auto command = committed_.top();
    command->undo(editor_state);
    undone_commands_.push(command);
    committed_.pop();
  }

  bool can_redo() const {
    return !undone_commands_.empty();
  }

  void redo(Out<EditorState> editor_state) {
    if (undone_commands_.empty()) {
      return;
    }

    const auto command = undone_commands_.top();
    command->commit(editor_state);
    committed_.push(command);
    undone_commands_.pop();
  }

 private:
  std::stack<std::shared_ptr<Command>> committed_;
  std::stack<std::shared_ptr<Command>> undone_commands_;
};
}  // namespace simulation
}  // namespace jcc