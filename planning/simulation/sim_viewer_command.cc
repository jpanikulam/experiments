#include "planning/simulation/sim_viewer_command.hh"

// TODO: Why is pymake failing to recurse dependencies?
#include <yaml-cpp/yaml.h>
namespace jcc {
namespace simulation {
std::vector<std::shared_ptr<Command>> load_commands(const YAML::Node& node) {
  std::vector<std::shared_ptr<Command>> commands;
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
    const YAML::Node& node = *it;

    const std::string cmd_type = node["command_type"].as<std::string>();
    if (cmd_type == "CreateElementCommand") {
      auto cmd = std::make_shared<CreateElementCommand>();
      cmd->load(node);
      commands.push_back(cmd);
    } else if (cmd_type == "CreateRobotCommand") {
      auto cmd = std::make_shared<CreateRobotCommand>();
      cmd->load(node);
      commands.push_back(cmd);
    }
  }
  return commands;
}

YAML::Node save_commands(const std::vector<std::shared_ptr<Command>>& commands) {
  YAML::Node everything;
  for (const auto& cmd : commands) {
    everything.push_back(cmd->save());
  }
  return everything;
}
}  // namespace simulation
}  // namespace jcc