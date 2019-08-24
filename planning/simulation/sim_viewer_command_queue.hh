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
  }

  void undo() {
    undone_commands_.push(committed_.top());
    committed_.pop();
  }

  void redo(Out<EditorState> editor_state) {
    commit(undone_commands_.top(), editor_state);
    undone_commands_.pop();
  }

 private:
  std::stack<std::shared_ptr<Command>> committed_;
  std::stack<std::shared_ptr<Command>> undone_commands_;
};
}  // namespace simulation
}  // namespace jcc