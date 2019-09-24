#pragma once
#include <cstddef>
namespace planning {
namespace drifter {
struct Debug {
  bool visualize_last_state = false;
  bool show_cost = true;
  bool enable_look_target = false;
  bool show_trajectory = true;
  bool convenience_flag = true;
};
struct Optimization {
  int max_iterations = 15;
  double min_state_damping = 1.0;
  double min_ctrl_damping = 1.0;
  double qxx_min_eigenvalue = 1.0;
  double quu_min_eigenvalue = 1.0;
};
struct Bounds {
  double v_max_weight = 250.0;
  double max_speed = 15.0;
  double min_speed = -10.0;
  double max_accel = 0.5;
  double phi_weight = 0.1;
  double phi_max = 3.0;
  double phi_max_bound_weight = 75.0;
};
struct Goal {
  double goal_weight = 1.0;
  double terminal_vel_weight = 10.0;
  double pointing_weight = 4.0;
  double path_weight = 1.0;
  double path_margin = 0.2;
  bool enable_path = false;
};
struct Obstacles {
  double avoid_weight = 3.0;
};
struct Control {
  double acceleration_weight = 0.1;
  double phidot_weight = 1.0;
  double phi_dot_max = 1.5;
  double phi_dot_max_bound_weight = 75.0;
};
struct PlannerConfiguration {
  Debug debug;
  Optimization optimization;
  Bounds bounds;
  Goal goal;
  Obstacles obstacles;
  Control control;
};
} // namespace drifter
} // namespace planning
