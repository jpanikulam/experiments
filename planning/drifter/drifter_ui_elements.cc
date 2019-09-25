#include "planning/drifter/drifter_ui_elements.hh"
#include "planning/simulation/sim_viewer_palette.hh"
#include "third_party/imgui/imgui.h"
namespace planning {
namespace drifter {
void show_menu(Out<PlannerConfiguration> menu) {  ImGui::Begin("PlannerConfiguration");
  if (ImGui::Button("Reset All")) {
    *menu = PlannerConfiguration{};  }
  if (ImGui::CollapsingHeader("Debug")) {
    if (ImGui::Button("Reset Debug")) {
      menu->debug = Debug{};
    }
    ImGui::Checkbox("Visualize Last State Cost", &menu->debug.visualize_last_state);
    ImGui::Checkbox("Show Cost", &menu->debug.show_cost);
    ImGui::Checkbox("Enable Look Target", &menu->debug.enable_look_target);
    ImGui::Checkbox("Show Trajectory", &menu->debug.show_trajectory);
    ImGui::Checkbox("Set Convenient Flag", &menu->debug.convenience_flag);
  }

  if (ImGui::CollapsingHeader("Optimization")) {
    if (ImGui::Button("Reset Optimization")) {
      menu->optimization = Optimization{};
    }
if (ImGui::Button("Reset##Max Iterations")) {
  menu->optimization.max_iterations = 15;
}
ImGui::SameLine();
    ImGui::SliderInt("Max Iterations", &(menu->optimization.max_iterations), 0, 100);
if (ImGui::Button("Reset##Minimum State Damping")) {
  menu->optimization.min_state_damping = 1.0;
}
ImGui::SameLine();
    float min_state_damping_tmp = static_cast<float>((menu->optimization.min_state_damping));
    ImGui::SliderFloat("Minimum State Damping", &min_state_damping_tmp, 1e-05, 10.0);
    menu->optimization.min_state_damping = static_cast<double>(min_state_damping_tmp);
if (ImGui::Button("Reset##Minimum Control Damping")) {
  menu->optimization.min_ctrl_damping = 1.0;
}
ImGui::SameLine();
    float min_ctrl_damping_tmp = static_cast<float>((menu->optimization.min_ctrl_damping));
    ImGui::SliderFloat("Minimum Control Damping", &min_ctrl_damping_tmp, 1e-05, 10.0);
    menu->optimization.min_ctrl_damping = static_cast<double>(min_ctrl_damping_tmp);
if (ImGui::Button("Reset##State Minimum Eigenvalue")) {
  menu->optimization.qxx_min_eigenvalue = 1.0;
}
ImGui::SameLine();
    float qxx_min_eigenvalue_tmp = static_cast<float>((menu->optimization.qxx_min_eigenvalue));
    ImGui::SliderFloat("State Minimum Eigenvalue", &qxx_min_eigenvalue_tmp, 1e-05, 1.0);
    menu->optimization.qxx_min_eigenvalue = static_cast<double>(qxx_min_eigenvalue_tmp);
if (ImGui::Button("Reset##Control Minimum Eigenvalue")) {
  menu->optimization.quu_min_eigenvalue = 1.0;
}
ImGui::SameLine();
    float quu_min_eigenvalue_tmp = static_cast<float>((menu->optimization.quu_min_eigenvalue));
    ImGui::SliderFloat("Control Minimum Eigenvalue", &quu_min_eigenvalue_tmp, 1e-05, 1.0);
    menu->optimization.quu_min_eigenvalue = static_cast<double>(quu_min_eigenvalue_tmp);
  }

  if (ImGui::CollapsingHeader("Bounds")) {
    if (ImGui::Button("Reset Bounds")) {
      menu->bounds = Bounds{};
    }
if (ImGui::Button("Reset##Speed Bound Weight")) {
  menu->bounds.v_max_weight = 250.0;
}
ImGui::SameLine();
    float v_max_weight_tmp = static_cast<float>((menu->bounds.v_max_weight));
    ImGui::SliderFloat("Speed Bound Weight", &v_max_weight_tmp, 10.0, 500.0);
    menu->bounds.v_max_weight = static_cast<double>(v_max_weight_tmp);
if (ImGui::Button("Reset##Maximum Speed (m/s)")) {
  menu->bounds.max_speed = 15.0;
}
ImGui::SameLine();
    float max_speed_tmp = static_cast<float>((menu->bounds.max_speed));
    ImGui::SliderFloat("Maximum Speed (m/s)", &max_speed_tmp, 0.0, 15.0);
    menu->bounds.max_speed = static_cast<double>(max_speed_tmp);
if (ImGui::Button("Reset##Minimum Speed (m/s)")) {
  menu->bounds.min_speed = -10.0;
}
ImGui::SameLine();
    float min_speed_tmp = static_cast<float>((menu->bounds.min_speed));
    ImGui::SliderFloat("Minimum Speed (m/s)", &min_speed_tmp, -10.0, 0.0);
    menu->bounds.min_speed = static_cast<double>(min_speed_tmp);
if (ImGui::Button("Reset##Maximum Acceleration (m/s^2)")) {
  menu->bounds.max_accel = 0.5;
}
ImGui::SameLine();
    float max_accel_tmp = static_cast<float>((menu->bounds.max_accel));
    ImGui::SliderFloat("Maximum Acceleration (m/s^2)", &max_accel_tmp, 0.0, 1.0);
    menu->bounds.max_accel = static_cast<double>(max_accel_tmp);
if (ImGui::Button("Reset##double")) {
  menu->bounds.phi_weight = 0.1;
}
ImGui::SameLine();
    float phi_weight_tmp = static_cast<float>((menu->bounds.phi_weight));
    ImGui::SliderFloat("double", &phi_weight_tmp, 0.0, 25.0);
    menu->bounds.phi_weight = static_cast<double>(phi_weight_tmp);
if (ImGui::Button("Reset##double")) {
  menu->bounds.phi_max = 3.0;
}
ImGui::SameLine();
    float phi_max_tmp = static_cast<float>((menu->bounds.phi_max));
    ImGui::SliderFloat("double", &phi_max_tmp, 0.0, 5.0);
    menu->bounds.phi_max = static_cast<double>(phi_max_tmp);
if (ImGui::Button("Reset##double")) {
  menu->bounds.phi_max_bound_weight = 75.0;
}
ImGui::SameLine();
    float phi_max_bound_weight_tmp = static_cast<float>((menu->bounds.phi_max_bound_weight));
    ImGui::SliderFloat("double", &phi_max_bound_weight_tmp, 0.0, 250.0);
    menu->bounds.phi_max_bound_weight = static_cast<double>(phi_max_bound_weight_tmp);
  }

  if (ImGui::CollapsingHeader("Goal")) {
    if (ImGui::Button("Reset Goal")) {
      menu->goal = Goal{};
    }
if (ImGui::Button("Reset##Goal Weight")) {
  menu->goal.goal_weight = 1.0;
}
ImGui::SameLine();
    float goal_weight_tmp = static_cast<float>((menu->goal.goal_weight));
    ImGui::SliderFloat("Goal Weight", &goal_weight_tmp, 0.0, 10.0);
    menu->goal.goal_weight = static_cast<double>(goal_weight_tmp);
if (ImGui::Button("Reset##Terminal Velocity Weight")) {
  menu->goal.terminal_vel_weight = 10.0;
}
ImGui::SameLine();
    float terminal_vel_weight_tmp = static_cast<float>((menu->goal.terminal_vel_weight));
    ImGui::SliderFloat("Terminal Velocity Weight", &terminal_vel_weight_tmp, 0.0, 20.0);
    menu->goal.terminal_vel_weight = static_cast<double>(terminal_vel_weight_tmp);
if (ImGui::Button("Reset##Pointing Target Weight")) {
  menu->goal.pointing_weight = 4.0;
}
ImGui::SameLine();
    float pointing_weight_tmp = static_cast<float>((menu->goal.pointing_weight));
    ImGui::SliderFloat("Pointing Target Weight", &pointing_weight_tmp, 0.0, 20.0);
    menu->goal.pointing_weight = static_cast<double>(pointing_weight_tmp);
if (ImGui::Button("Reset##Path Weight")) {
  menu->goal.path_weight = 1.0;
}
ImGui::SameLine();
    float path_weight_tmp = static_cast<float>((menu->goal.path_weight));
    ImGui::SliderFloat("Path Weight", &path_weight_tmp, 0.0, 20.0);
    menu->goal.path_weight = static_cast<double>(path_weight_tmp);
if (ImGui::Button("Reset##Path Margin (m)")) {
  menu->goal.path_margin = 0.2;
}
ImGui::SameLine();
    float path_margin_tmp = static_cast<float>((menu->goal.path_margin));
    ImGui::SliderFloat("Path Margin (m)", &path_margin_tmp, 0.0, 1.0);
    menu->goal.path_margin = static_cast<double>(path_margin_tmp);
    ImGui::Checkbox("Enable Path", &menu->goal.enable_path);
  }

  if (ImGui::CollapsingHeader("Obstacles")) {
    if (ImGui::Button("Reset Obstacles")) {
      menu->obstacles = Obstacles{};
    }
if (ImGui::Button("Reset##Avoid Area Weight")) {
  menu->obstacles.avoid_weight = 3.0;
}
ImGui::SameLine();
    float avoid_weight_tmp = static_cast<float>((menu->obstacles.avoid_weight));
    ImGui::SliderFloat("Avoid Area Weight", &avoid_weight_tmp, 0.0, 10.0);
    menu->obstacles.avoid_weight = static_cast<double>(avoid_weight_tmp);
  }

  if (ImGui::CollapsingHeader("Control")) {
    if (ImGui::Button("Reset Control")) {
      menu->control = Control{};
    }
if (ImGui::Button("Reset##Acceleration Weight")) {
  menu->control.acceleration_weight = 0.0;
}
ImGui::SameLine();
    float acceleration_weight_tmp = static_cast<float>((menu->control.acceleration_weight));
    ImGui::SliderFloat("Acceleration Weight", &acceleration_weight_tmp, 0.0, 2.0);
    menu->control.acceleration_weight = static_cast<double>(acceleration_weight_tmp);
if (ImGui::Button("Reset##Steering Angle Rate Weight")) {
  menu->control.phidot_weight = 1.0;
}
ImGui::SameLine();
    float phidot_weight_tmp = static_cast<float>((menu->control.phidot_weight));
    ImGui::SliderFloat("Steering Angle Rate Weight", &phidot_weight_tmp, 0.0, 10.0);
    menu->control.phidot_weight = static_cast<double>(phidot_weight_tmp);
if (ImGui::Button("Reset##Maximum Value of dPhi/dt")) {
  menu->control.phi_dot_max = 1.5;
}
ImGui::SameLine();
    float phi_dot_max_tmp = static_cast<float>((menu->control.phi_dot_max));
    ImGui::SliderFloat("Maximum Value of dPhi/dt", &phi_dot_max_tmp, 0.0, 3.0);
    menu->control.phi_dot_max = static_cast<double>(phi_dot_max_tmp);
if (ImGui::Button("Reset##Weight for Enforcing Maximum Value of dPhi/dt")) {
  menu->control.phi_dot_max_bound_weight = 75.0;
}
ImGui::SameLine();
    float phi_dot_max_bound_weight_tmp = static_cast<float>((menu->control.phi_dot_max_bound_weight));
    ImGui::SliderFloat("Weight for Enforcing Maximum Value of dPhi/dt", &phi_dot_max_bound_weight_tmp, 0.0, 250.0);
    menu->control.phi_dot_max_bound_weight = static_cast<double>(phi_dot_max_bound_weight_tmp);
  }
  ImGui::End();
}} // namespace drifter
} // namespace planning
