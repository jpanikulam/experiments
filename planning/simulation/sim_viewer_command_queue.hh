#include <stack>

#include "out.hh"
#include "planning/simulation/sim_viewer_command.hh"
#include "planning/simulation/sim_viewer_types.hh"

namespace jcc {
namespace simulation {

class CommandQueue {
 public:
  void commit(const Command& command, Out<EditorState> editor_state) {
    command.commit(editor_state);
  }

  void undo() {
    undone_commands_.push(commands_.top());
    commands_.pop();
  }

  void redo() {
    commit(undone_commands_.top());
    undone_commands_.pop();
  }

 private:
  std::stack<std::shared_ptr<Command>> executed_;
  std::stack<std::shared_ptr<Command>> undone_commands_;
};
}  // namespace simulation
}  // namespace jcc