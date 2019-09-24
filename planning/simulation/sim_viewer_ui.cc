#include "planning/simulation/sim_viewer_ui.hh"

#include "logging/assert.hh"
#include "planning/simulation/sim_viewer_easy_commands.hh"
#include "planning/simulation/sim_viewer_placement.hh"

#include "third_party/imgui/imgui.h"

//%deps(stdc++fs)
#include <experimental/filesystem>

namespace jcc {
namespace simulation {

namespace {

int pushcolor(int c) {
  if (c == 0) {
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.1f, 0.6f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.1f, 0.8f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.05f, 0.95f, 0.05f));

    return 3;
  } else if (c == 1) {
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.6f, 0.1f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.8f, 0.1f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.95f, 0.05f, 0.05f));
    return 3;
  }

  if (c == 2) {
    ImGui::PushStyleColor(ImGuiCol_TextDisabled, (ImVec4)ImColor(0.6f, 0.1f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_Text, (ImVec4)ImColor(0.8f, 0.1f, 0.1f));
    return 2;
  } else if (c == 3) {
    ImGui::PushStyleColor(ImGuiCol_TextDisabled, (ImVec4)ImColor(0.1f, 0.6f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_Text, (ImVec4)ImColor(0.1f, 0.8f, 0.1f));
    return 2;
  }
  // TODO!
  return 1;
}

}  // namespace

bool load_manifest_modal(const EditorState& editor_state, Out<MainMenuState> menu_state) {
  namespace fs = std::experimental::filesystem;
  if (menu_state->manifest_stale) {
    menu_state->manifest_elements.clear();
    const YAML::Node node =
        YAML::LoadFile("/home/jacob/repos/experiments/sims/manifest.yaml");

    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      const std::string key = it->second["category"].as<std::string>();

      const auto sims_node = it->second["sims"];
      for (YAML::const_iterator sim_it = sims_node.begin(); sim_it != sims_node.end();
           ++sim_it) {
        ManifestEntry entry;

        entry.name = (*sim_it)["name"].as<std::string>();
        entry.passing = (*sim_it)["passing"].as<bool>();
        entry.filename = (*sim_it)["file"].as<std::string>();

        entry.allowed = it->second["allowed"].as<bool>();
        menu_state->manifest_elements[key].push_back(entry);
      }
    }
    menu_state->manifest_stale = false;
  }

  bool complete = false;

  ImGui::SetNextWindowSizeConstraints(ImVec2(500, 100), ImVec2(10000, 10000));
  if (ImGui::BeginPopupModal("Manifest Modal", NULL)) {
    ImGui::Columns(2);
    for (const auto& p : menu_state->manifest_elements) {
      for (const auto& entry : p.second) {
        int flags = ImGuiSelectableFlags_SpanAllColumns;
        if (!entry.allowed) {
          flags |= ImGuiSelectableFlags_Disabled;
        }
        if (entry.passing) {
          pushcolor(3);
          if (ImGui::Selectable(entry.name.c_str(), false, flags)) {
            menu_state->new_file = true;
            menu_state->new_file_name = entry.filename;
            complete = true;
            ImGui::CloseCurrentPopup();
          }
          ImGui::PopStyleColor(2);
        } else {
          pushcolor(2);
          if (ImGui::Selectable(entry.name.c_str(), false, flags)) {
            menu_state->new_file = true;
            menu_state->new_file_name = entry.filename;
            complete = true;
            ImGui::CloseCurrentPopup();
          }
          ImGui::PopStyleColor(2);
        }
        ImGui::NextColumn();
        const std::string category = p.first;
        ImGui::Text("%s", category.c_str());
        ImGui::NextColumn();
      }
    }
    ImGui::EndPopup();
  }
  return complete;
}

void create_main_menu(const EditorState& editor_state, Out<MainMenuState> menu_state) {
  menu_state->cmd_queue_update = EditorCommand::None;

  bool save_as_modal = false;
  bool manifest_modal = false;
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("New")) {
        jcc::Warning() << "Creating new sim..." << std::endl;
      }
      if (ImGui::MenuItem("Open", "Ctrl+O")) {
        jcc::Warning() << "Opening sim..." << std::endl;
        manifest_modal = true;
        menu_state->manifest_stale = true;
      }
      if (ImGui::BeginMenu("Open Recent")) {
        ImGui::MenuItem("easy-1.sim");
        ImGui::MenuItem("easy-2.sim");
        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Save", "Ctrl+S")) {
        if (editor_state.filename == "") {
          save_as_modal = true;
        } else {
          menu_state->cmd_queue_update = EditorCommand::Save;
        }
      }
      if (ImGui::MenuItem("Save As..")) {
        save_as_modal = true;
      }

      ImGui::Separator();
      if (ImGui::BeginMenu("Options")) {
        ImGui::EndMenu();
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "CTRL+Z", false, editor_state.can_undo)) {
        menu_state->cmd_queue_update = EditorCommand::Undo;
      }
      if (ImGui::MenuItem("Redo", "CTRL+Y", false, editor_state.can_redo)) {
        menu_state->cmd_queue_update = EditorCommand::Redo;
      }  // Disabled item
      ImGui::Separator();
      if (ImGui::MenuItem("Cut", "CTRL+X", false, false)) {
      }
      if (ImGui::MenuItem("Copy", "CTRL+C", false, false)) {
      }
      if (ImGui::MenuItem("Paste", "CTRL+V", false, false)) {
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Controls")) {
      ImGui::Checkbox("Snap to Grid", &menu_state->snap_to_grid);
      ImGui::SliderFloat("Sim Speed Scaling", &menu_state->sim_speed, 0.0f, 5.0f);
      ImGui::EndMenu();
    }
  }
  ImGui::EndMainMenuBar();

  if (save_as_modal) {
    ImGui::OpenPopup("Save As Modal");
  }

  if (ImGui::BeginPopupModal("Save As Modal", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("Enter a name for your file...");
    if (ImGui::InputText("Filename",
                         menu_state->filename_entry_buffer,
                         64,
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
      menu_state->cmd_queue_update = EditorCommand::Save;
      ImGui::CloseCurrentPopup();
    }
    ImGui::Separator();
    ImGui::EndPopup();
  }

  if (manifest_modal) {
    ImGui::OpenPopup("Manifest Modal");
  }

  if (load_manifest_modal(editor_state, menu_state)) {
    menu_state->cmd_queue_update = EditorCommand::Open;
  }
}

jcc::Optional<std::shared_ptr<Command>> menus_new_object(
    const EditorState& editor_state, Out<TaskPopupState> task_popup_state) {
  jcc::Optional<std::shared_ptr<Command>> cmd = std::nullopt;
  const bool not_hovered = task_popup_state->hovered == -1;

  if (not_hovered) {
    if (ImGui::BeginMenu("Test Zones")) {
      const int pops0 = pushcolor(0);
      if (ImGui::Button("Go Zone")) {
        cmd = {create_element(
            editor_state, ElementType::GoZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      ImGui::PopStyleColor(pops0);

      const int pops1 = pushcolor(1);
      if (ImGui::Button("No-Go Zone")) {
        cmd = {create_element(
            editor_state, ElementType::NoGoZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      ImGui::PopStyleColor(pops1);

      if (ImGui::Button("Must-Look Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("Speed Limit Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("IMU Loss Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("Camera Loss Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("IMU Latency Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("Camera Latency Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      if (ImGui::Button("Wind Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndMenu();
    }
  } else {
    if (ImGui::BeginMenu("Edit Properties")) {
      // TODO
      static float orientation = 0.0;
      if (ImGui::SliderFloat("Orientation", &orientation, 0.0f, 5.0f)) {
      }

      const Eigen::Vector3f size_f = editor_state.elements.at(task_popup_state->hovered)
                                         .properties.at("size")
                                         .cast<float>();
      float size[3] = {size_f.x(), size_f.y(), size_f.z()};
      if (ImGui::SliderFloat3("Size", size, 0.0f, 5.0f)) {
        const jcc::Vec3 size_vec_unscaled(size[0], size[1], size[2]);

        const jcc::Vec3 size_vec = editor_state.config.snap_to_grid
                                       ? snap_to_grid(size_vec_unscaled)
                                       : size_vec_unscaled;
        if ((size_vec.cast<float>() - size_f).squaredNorm() > 0.1) {
          cmd = {create_modification(
              task_popup_state->hovered, editor_state, "size", size_vec)};
        }
      }
      ImGui::EndMenu();
    }
  }

  if (ImGui::BeginMenu("Robots")) {
    ImGui::MenuItem("Hover Jet", nullptr, false, false);
    if (ImGui::MenuItem("Drifter", nullptr, false, !editor_state.any_robot_placed)) {
      cmd = {
          create_robot(editor_state, "Drifter", task_popup_state->creation_world_point)};
    }

    if (ImGui::MenuItem("Grocery Scanner", nullptr, false, !editor_state.any_robot_placed)) {
      cmd = {create_robot(
          editor_state, "Bossa Nova", task_popup_state->creation_world_point)};
    }
    if (ImGui::MenuItem("Scrubber", nullptr, false, !editor_state.any_robot_placed)) {
      cmd = {
          create_robot(editor_state, "Scrubber", task_popup_state->creation_world_point)};
    }

    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Features")) {
    if (ImGui::MenuItem("Shelf - Forward")) {
      cmd = {create_object(editor_state,
                           ElementType::NoGoZone,
                           "One Shelf+X",
                           task_popup_state->creation_world_point)};
    }
    if (ImGui::MenuItem("Shelf - Back")) {
      cmd = {create_object(editor_state,
                           ElementType::NoGoZone,
                           "One Shelf-X",
                           task_popup_state->creation_world_point)};
    }
    if (ImGui::MenuItem("Store")) {
      cmd = {create_object(editor_state,
                           ElementType::Object,
                           "Store",
                           task_popup_state->creation_world_point)};
    }

    ImGui::MenuItem("Jet Obstacle-Occluder", nullptr, false, false);
    ImGui::MenuItem("Jet Occluder", nullptr, false, false);
    ImGui::MenuItem("Jet Landing Pad", nullptr, false, false);
    ImGui::EndMenu();
  }

  return cmd;
}

/*jcc::Optional<std::shared_ptr<Command>> menus_modify_object(
    const EditorState& editor_state, Out<TaskPopupState> task_popup_state) {
  jcc::Optional<std::shared_ptr<Command>> cmd = std::nullopt;

  JASSERT_GE(editor_state.hovered, 0, "Something must be hovered to modify");
  const auto& element = editor_state.elements.at(editor_state.hovered);

  if (element.element_type != ElementType::Object) {
  }

  const bool not_hovered = editor_state.hovered != -1;

  if (not_hovered) {
    if (ImGui::BeginMenu("Change Zone Type")) {
      ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.1f, 0.6f, 0.1f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.1f, 0.8f, 0.1f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.05f, 0.95f,
0.05f)); if (ImGui::Button("Go Zone")) { cmd = {create_element( editor_state,
ElementType::GoZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      ImGui::PopStyleColor(3);
      ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.6f, 0.1f, 0.1f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.8f, 0.1f, 0.1f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.95f, 0.05f,
0.05f)); if (ImGui::Button("No-Go Zone")) { cmd = {create_element( editor_state,
ElementType::NoGoZone, task_popup_state->creation_world_point)};
        ImGui::CloseCurrentPopup();
      }
      ImGui::PopStyleColor(3);

      if (ImGui::Button("Must-Look Zone")) {
        cmd = {create_element(
            editor_state, ElementType::LookZone,
task_popup_state->creation_world_point)}; ImGui::CloseCurrentPopup();
      }
      ImGui::EndMenu();
    }
  }
  if (ImGui::BeginMenu("Robots")) {
    ImGui::MenuItem("Hover Jet", nullptr, false, false);
    ImGui::MenuItem("Drifter", nullptr, false, !editor_state.any_robot_placed);
    ImGui::MenuItem("Bossa Nova", nullptr, false, !editor_state.any_robot_placed);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Features")) {
    if (ImGui::MenuItem("Shelf")) {
      cmd = {create_object(
          editor_state, ElementType::Object, task_popup_state->creation_world_point)};
    }
    ImGui::MenuItem("Jet Obstacle-Occluder", nullptr, false, false);
    ImGui::MenuItem("Jet Occluder", nullptr, false, false);
    ImGui::MenuItem("Jet Landing Pad", nullptr, false, false);
    ImGui::EndMenu();
  }

  return cmd;
}
*/
jcc::Optional<std::shared_ptr<Command>> create_task_popup(
    const EditorState& editor_state, Out<TaskPopupState> task_popup_state) {
  jcc::Optional<std::shared_ptr<Command>> cmd = std::nullopt;

  const bool ctrl_held = (ImGui::GetIO().KeyCtrl);

  if (!ImGui::IsItemHovered() && ImGui::IsMouseClicked(1) && ctrl_held) {
    ImGui::OpenPopup("Add Component");
  }

  if (ImGui::BeginPopup("Add Component")) {
    if (task_popup_state->is_new) {
      task_popup_state->creation_world_point = editor_state.world_click_pos;
      task_popup_state->is_new = false;
      task_popup_state->hovered = editor_state.hovered;
    }

    // if (editor_state.hovered != -1) {
    // cmd = menus_modify_object(editor_state, task_popup_state);
    // } else {
    cmd = menus_new_object(editor_state, task_popup_state);
    // }
    ImGui::EndPopup();
  } else {
    task_popup_state->is_new = true;
    task_popup_state->hovered = -1;
  }

  return cmd;
}

void debug_window() {
  const auto& io = ImGui::GetIO();
  ImGuiColorEditFlags misc_flags = 0;
  ImVec4 color(0.0, 0.0, 0.0, 0.0);
  ImGui::ColorEdit3("MyColor##1", (float*)&color, misc_flags);
  ImGui::Text("Keys mods: %s%s%s%s",
              io.KeyCtrl ? "CTRL " : "",
              io.KeyShift ? "SHIFT " : "",
              io.KeyAlt ? "ALT " : "",
              io.KeySuper ? "SUPER " : "");
}

}  // namespace simulation
}  // namespace jcc