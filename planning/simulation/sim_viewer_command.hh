#pragma once

#include "eigen.hh"
#include "out.hh"

#include "planning/simulation/sim_viewer_types.hh"

// TODO
#include <iostream>

// TODO
#include <yaml-cpp/yaml.h>
#include "planning/simulation/yml_matrix.hh"

namespace jcc {
namespace simulation {

struct Command {
  virtual ~Command() = default;
  virtual void commit(Out<EditorState> state) = 0;
  virtual void undo(Out<EditorState> state) = 0;

  std::string desc;
  virtual std::string description() {
    return desc;
  };

  virtual YAML::Node save() const = 0;
  virtual void load(const YAML::Node&) = 0;
};

struct CreateElementCommand : public Command {
  int id;
  ElementType element_type;
  SE3 world_from_element;
  std::map<std::string, jcc::Vec3> initial_properties;
  std::string model;
  std::string desc;

  void commit(Out<EditorState> state) {
    Element obj;
    obj.element_type = element_type;
    obj.world_from_element = world_from_element;
    obj.model = model;
    obj.properties = initial_properties;
    state->elements[id] = obj;
  }

  void undo(Out<EditorState> state) {
    state->elements.erase(id);
  }

  YAML::Node save() const {
    YAML::Node node;

    node["id"] = id;
    node["element_type"] = static_cast<int>(element_type);
    auto wel = node["world_from_element"];
    se3_to_yaml(wel, world_from_element);
    auto props_node = node["initial_properties"];
    for (const auto& p : initial_properties) {
      set_matrix(props_node, p.first, p.second);
    }
    node["model"] = model;
    node["desc"] = desc;

    node["command_type"] = "CreateElementCommand";
    return node;
  }

  void load(const YAML::Node& node) {
    id = node["id"].as<int>();
    element_type = static_cast<ElementType>(node["element_type"].as<int>());
    world_from_element = yaml_to_se3(node["world_from_element"]);

    const YAML::Node props_node = node["initial_properties"];
    for (YAML::const_iterator it = props_node.begin(); it != props_node.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      initial_properties[key] = read_matrix<3, 1>(it->second);
    }
  }
};

// TODO: Generate these
//
// - extents : vector
// - success time: scalar
struct ModifyElementCommand : public Command {
  int id;
  std::string property;
  jcc::Vec3 value;
  std::string desc;

  void commit(Out<EditorState> state) {
    if (state->elements[id].properties.count(property)) {
      previous_value_ = state->elements[id].properties[property];
      had_previous_value_ = true;
    }
    state->elements[id].properties[property] = value;
  }

  void undo(Out<EditorState> state) {
    if (had_previous_value_) {
      state->elements[id].properties[property] = previous_value_;
    } else {
      state->elements[id].properties.erase(property);
    }
  }

  YAML::Node save() const {
    YAML::Node node;
    node["id"] = id;
    node["property"] = property;
    set_matrix(node, "value", value);
    node["desc"] = desc;

    node["had_previous_value"] = had_previous_value_;
    set_matrix(node, "previous_value", previous_value_);

    return node;
  }
  void load(const YAML::Node&) {
    YAML::Node node;
    id = node["id"].as<int>();
    property = node["property"].as<std::string>();

    // const YAML::Node rnode = node["value"];
    value = read_matrix<3, 1>(node["value"]);
    desc = node["desc"].as<std::string>();

    had_previous_value_ = node["had_previous_value"].as<bool>();
    previous_value_ = read_matrix<3, 1>(node["previous_value"]);
  }

 private:
  bool had_previous_value_ = false;
  jcc::Vec3 previous_value_;
};

struct CreateRobotCommand : public Command {
  int id;
  SE3 world_from_robot;
  std::map<std::string, jcc::Vec3> initial_properties;
  std::string model;

  std::string desc;

  void commit(Out<EditorState> state) {
    Robot robot;
    robot.world_from_robot = world_from_robot;
    robot.model = model;
    robot.properties = initial_properties;
    state->robot = robot;
    state->any_robot_placed = true;
  }

  void undo(Out<EditorState> state) {
    state->robot = {};
    state->robot.model = "UNKNOWN";
    state->any_robot_placed = false;
  }

  YAML::Node save() const {
    YAML::Node node;

    node["id"] = id;
    auto wel = node["world_from_robot"];
    se3_to_yaml(wel, world_from_robot);
    auto props_node = node["initial_properties"];
    for (const auto& p : initial_properties) {
      set_matrix(props_node, p.first, p.second);
    }
    node["model"] = model;
    node["desc"] = desc;

    node["command_type"] = "CreateRobotCommand";
    return node;
  }

  void load(const YAML::Node& node) {
    id = node["id"].as<int>();
    world_from_robot = yaml_to_se3(node["world_from_robot"]);

    const YAML::Node props_node = node["initial_properties"];
    for (YAML::const_iterator it = props_node.begin(); it != props_node.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      initial_properties[key] = read_matrix<3, 1>(it->second);
    }
  }
};

std::vector<std::shared_ptr<Command>> load_commands(const YAML::Node& node);

YAML::Node save_commands(const std::vector<std::shared_ptr<Command>>& commands);

}  // namespace simulation
}  // namespace jcc