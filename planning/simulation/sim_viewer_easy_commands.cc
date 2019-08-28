#include "planning/simulation/sim_viewer_easy_commands.hh"

#include "planning/simulation/sim_viewer_placement.hh"

namespace jcc {
namespace simulation {
std::shared_ptr<Command> create_robot(const EditorState& editor_state,
                                      const std::string& robot_name,
                                      const jcc::Vec3& world_click_pos) {
  const jcc::Vec3 world_pt =
      editor_state.config.snap_to_grid ? snap_to_grid(world_click_pos) : world_click_pos;

  const SE3 world_from_robot = SE3(SO3(), world_pt);
  auto cmd = std::make_shared<CreateRobotCommand>();
  cmd->id = 1;
  cmd->world_from_robot = world_from_robot;
  cmd->model = robot_name;
  cmd->desc = "Create Robot: " + robot_name;

  // cmd->initial_properties = {{}};
  return cmd;
}

std::shared_ptr<Command> create_element(const EditorState& editor_state,
                                        const ElementType element_type,
                                        const jcc::Vec3& world_click_pos) {
  const int id = static_cast<int>(editor_state.elements.size());

  const jcc::Vec3 world_pt =
      editor_state.config.snap_to_grid ? snap_to_grid(world_click_pos) : world_click_pos;

  const SE3 world_from_element = SE3(SO3(), world_pt);
  auto cmd = std::make_shared<CreateElementCommand>();
  cmd->id = id;
  cmd->element_type = element_type;
  cmd->world_from_element = world_from_element;
  cmd->desc = "Create Object";
  cmd->initial_properties = {{"size", jcc::Vec3::Ones()}};
  return cmd;
}

std::shared_ptr<Command> create_modification(int id,
                                             const EditorState& editor_state,
                                             const std::string& property,
                                             const jcc::Vec3& new_val) {
  auto cmd = std::make_shared<ModifyElementCommand>();
  cmd->id = id;
  cmd->property = property;
  cmd->value = new_val;
  cmd->desc = "Modify Object: " + std::to_string(id) + " " + property;

  return cmd;
}

std::shared_ptr<Command> create_object(const EditorState& editor_state,
                                       const ElementType element_type,
                                       const std::string& model,
                                       const jcc::Vec3& world_click_pos) {
  const int id = static_cast<int>(editor_state.elements.size());

  const jcc::Vec3 world_pt =
      editor_state.config.snap_to_grid ? snap_to_grid(world_click_pos) : world_click_pos;

  const SE3 world_from_element = SE3(SO3(), world_pt);
  auto cmd = std::make_shared<CreateElementCommand>();
  cmd->id = id;
  cmd->element_type = element_type;
  cmd->world_from_element = world_from_element;
  cmd->desc = "Create Object: " + model;
  cmd->model = model;
  cmd->initial_properties = {{"size", jcc::Vec3::Ones()}};
  return cmd;
}

}  // namespace simulation
}  // namespace jcc