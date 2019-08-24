#include "planning/simulation/sim_viewer_ui.hh"

#include "third_party/imgui/imgui.h"

#include "logging/assert.hh"

namespace jcc {
namespace simulation {

void create_main_menu(Out<MainMenuState> menu_state) {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("New")) {
        jcc::Warning() << "Creating new sim..." << std::endl;
      }
      if (ImGui::MenuItem("Open", "Ctrl+O")) {
        jcc::Warning() << "Opening sim..." << std::endl;
      }
      if (ImGui::BeginMenu("Open Recent")) {
        ImGui::MenuItem("simple_jet.sim");
        ImGui::MenuItem("easy_jet.sim");
        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Save", "Ctrl+S")) {
        jcc::Warning() << "Saving..." << std::endl;
      }
      if (ImGui::MenuItem("Save As..")) {
      }

      ImGui::Separator();
      if (ImGui::BeginMenu("Options")) {
        ImGui::EndMenu();
      }

      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Edit")) {
      if (ImGui::MenuItem("Undo", "CTRL+Z")) {
      }
      if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {
      }  // Disabled item
      ImGui::Separator();
      if (ImGui::MenuItem("Cut", "CTRL+X")) {
      }
      if (ImGui::MenuItem("Copy", "CTRL+C")) {
      }
      if (ImGui::MenuItem("Paste", "CTRL+V")) {
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Controls")) {
      ImGui::SliderFloat("Sim Speed Scaling", &menu_state->sim_speed, 0.0f, 5.0f);
      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }
}

void create_task_popup(const EditorState& editor_state,
                       Out<TaskPopupState> task_popup_state) {
  if (!ImGui::IsItemHovered() && ImGui::IsMouseClicked(1)) {
    ImGui::OpenPopup("Add Component");
  }
  if (ImGui::BeginPopup("Add Component")) {
    ImGui::Text("Add Component");

    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.1f, 0.6f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.1f, 0.8f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.05f, 0.95f, 0.05f));
    if (ImGui::Button("Go Zone")) {
      const int id = static_cast<int>(editor_state.elements.size());

      ImGui::CloseCurrentPopup();
    }
    ImGui::PopStyleColor(3);

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor(0.6f, 0.1f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor(0.8f, 0.1f, 0.1f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor(0.95f, 0.05f, 0.05f));
    if (ImGui::Button("No-Go Zone")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::PopStyleColor(3);

    if (ImGui::Button("Must-Look Zone")) {
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

}  // namespace simulation
}  // namespace jcc