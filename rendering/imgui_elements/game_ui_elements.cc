#include "rendering/imgui_elements/game_ui_elements.hh"
#include "planning/simulation/sim_viewer_palette.hh"
#include "third_party/imgui/imgui.h"
namespace jcc {
void show_menu(Out<GameDebugConfiguration> menu) {  ImGui::Begin("GameDebugConfiguration");
  if (ImGui::Button("Reset All")) {
    *menu = GameDebugConfiguration{};  }
  if (ImGui::CollapsingHeader("Debug")) {
    if (ImGui::Button("Reset Debug")) {
      menu->debug = Debug{};
    }
{    ImGui::Checkbox("Use Normals", &menu->debug.use_normals);
}{    ImGui::Checkbox("Use Reflective Shadow Maps", &menu->debug.use_rsm);
}{    ImGui::Checkbox("Use Wireframe", &menu->debug.wireframe);
}{if (ImGui::Button("Reset##Theta")) {
  menu->debug.theta = 1.2;
}
ImGui::SameLine();
    float theta_tmp = static_cast<float>((menu->debug.theta));
    ImGui::SliderFloat("Theta", &theta_tmp, 0.0, 3.14);
    menu->debug.theta = static_cast<double>(theta_tmp);
}{if (ImGui::Button("Reset##Distance")) {
  menu->debug.d = 1.2;
}
ImGui::SameLine();
    float d_tmp = static_cast<float>((menu->debug.d));
    ImGui::SliderFloat("Distance", &d_tmp, 0.0, 5.0);
    menu->debug.d = static_cast<double>(d_tmp);
}{    const char* element_names[3] = {"GL_BACK", "GL_FRONT", "GL_FRONT_AND_BACK"};
    const char* current_element_name = (menu->debug.polygon_mode >= 0 && menu->debug.polygon_mode < 3) ? element_names[menu->debug.polygon_mode] : "Unknown";
    ImGui::SliderInt("Polygon Mode", &(menu->debug.polygon_mode), 0, 3 - 1, current_element_name);
}  }
  ImGui::End();
}} // namespace jcc
