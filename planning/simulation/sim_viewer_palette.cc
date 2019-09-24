#include "planning/simulation/sim_viewer_palette.hh"

#include "third_party/imgui/imgui.h"

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
