#include "rendering/imgui_elements/game_ui_elements.hh"
#include "planning/simulation/sim_viewer_palette.hh"
#include "third_party/imgui/imgui.h"
namespace jcc {
void show_menu(Out<GameDebugConfiguration> menu) {  ImGui::Begin("Game Debug");
  if (ImGui::Button("Reset All")) {
    *menu = GameDebugConfiguration{};  }
  if (ImGui::CollapsingHeader("Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Reset Debug")) {
      menu->debug = Debug{};
    }
{    ImGui::Checkbox("Use Wireframe", &menu->debug.wireframe);
}{if (ImGui::Button("Reset##Theta")) {
  menu->debug.theta = 3.14;
}
ImGui::SameLine();
    float theta_tmp = static_cast<float>((menu->debug.theta));
    ImGui::SliderFloat("Theta", &theta_tmp, 0.0, 3.14);
    menu->debug.theta = static_cast<double>(theta_tmp);
}{if (ImGui::Button("Reset##Distance")) {
  menu->debug.d = 0.438;
}
ImGui::SameLine();
    float d_tmp = static_cast<float>((menu->debug.d));
    ImGui::SliderFloat("Distance", &d_tmp, 0.0, 5.0);
    menu->debug.d = static_cast<double>(d_tmp);
}  }

  if (ImGui::CollapsingHeader("Shading", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Reset Shading")) {
      menu->shading = Shading{};
    }
{    ImGui::Checkbox("Enable shadows", &menu->shading.enable_shadows);
}{    ImGui::Checkbox("Enable our mysterious shader debug flag", &menu->shading.misc_debug);
}{    ImGui::Checkbox("Enable SRGB", &menu->shading.srgb);
}{    ImGui::Checkbox("Use Reflective Shadow Maps", &menu->shading.use_rsm);
}{    ImGui::Checkbox("Show light probe locations in red", &menu->shading.show_light_probes);
}  }
  ImGui::End();
}} // namespace jcc
