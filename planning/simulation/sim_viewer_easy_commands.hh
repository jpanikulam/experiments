#pragma once

#include "planning/simulation/sim_viewer_command.hh"
#include "planning/simulation/sim_viewer_types.hh"

#include "eigen.hh"

#include <memory>

namespace jcc {
namespace simulation {

std::shared_ptr<Command> create_robot(const EditorState& editor_state,
                                      const std::string& robot_name,
                                      const jcc::Vec3& world_click_pos);

std::shared_ptr<Command> create_element(const EditorState& editor_state,
                                        const ElementType element_type,
                                        const jcc::Vec3& world_click_pos);

std::shared_ptr<Command> create_modification(int id,
                                             const EditorState& editor_state,
                                             const std::string& property,
                                             const jcc::Vec3& new_val);

std::shared_ptr<Command> create_object(const EditorState& editor_state,
                                       const ElementType element_type,
                                       const std::string& model,
                                       const jcc::Vec3& world_click_pos);

}  // namespace simulation
}  // namespace jcc