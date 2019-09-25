from generate_imgui import MenuGenerator


def main():
    full_menu = MenuGenerator("PlannerConfiguration")

    # Debug
    debug = full_menu.add_submenu("Debug")
    debug.add_toggle("visualize_last_state", "Visualize Last State Cost", default="false")
    debug.add_toggle("visualize_last_state", "Visualize Last State Cost", default="false")
    debug.add_toggle("show_cost", "Show Cost", default="true")
    debug.add_toggle("enable_look_target", "Enable Look Target", default="false")
    debug.add_toggle("show_trajectory", "Show Trajectory", default="true")
    debug.add_toggle("convenience_flag", "Set Convenient Flag", default="true")

    # Optimization
    optimization = full_menu.add_submenu("Optimization")
    optimization.add_int_scalar("max_iterations", "Max Iterations", (0, 100), default="15")
    optimization.add_scalar('min_state_damping', 'Minimum State Damping', (1e-5, 10.0), "1.0")
    optimization.add_scalar('min_ctrl_damping', 'Minimum Control Damping', (1e-5, 10.0), "1.0")
    optimization.add_scalar('qxx_min_eigenvalue', 'State Minimum Eigenvalue', (1e-5, 1.0), "1.0")
    optimization.add_scalar('quu_min_eigenvalue', 'Control Minimum Eigenvalue', (1e-5, 1.0), "1.0")

    # Bounds
    bounds = full_menu.add_submenu("Bounds")
    bounds.add_scalar('v_max_weight', 'Speed Bound Weight', (10.0, 500.0), 250.0)
    bounds.add_scalar('max_speed', 'Maximum Speed (m/s)', (0.0, 15.0), 15.0)
    bounds.add_scalar('min_speed', 'Minimum Speed (m/s)', (-10.0, 0.0), -10.0)
    bounds.add_scalar('max_accel', 'Maximum Acceleration (m/s^2)', (0.0, 1.0), 0.5)
    bounds.add_scalar('phi_weight', 'double', (0.0, 25.0), 0.1)
    bounds.add_scalar('phi_max', 'double', (0.0, 5.0), 3.0)
    bounds.add_scalar('phi_max_bound_weight', 'double', (0.0, 250.0), 75.0)

    # Goal
    goals = full_menu.add_submenu("Goal")
    goals.add_scalar('goal_weight', 'Goal Weight', (0.0, 10.0), 1.0)
    goals.add_scalar('terminal_vel_weight', 'Terminal Velocity Weight', (0.0, 20.0), 10.0)
    goals.add_scalar('pointing_weight', 'Pointing Target Weight', (0.0, 20.0), 4.0)
    goals.add_scalar('path_weight', 'Path Weight', (0.0, 20.0), 1.0)
    goals.add_scalar('path_margin', 'Path Margin (m)', (0.0, 1.0), 0.2)
    goals.add_toggle('enable_path', 'Enable Path', "false")

    # Obstacles
    obstacles = full_menu.add_submenu("Obstacles")
    obstacles.add_scalar('avoid_weight', 'Avoid Area Weight', (0.0, 10.0), 3.0)

    # Control
    control = full_menu.add_submenu("Control")
    control.add_scalar('acceleration_weight', 'Acceleration Weight', (0.0, 2.0), 0.)
    control.add_scalar('phidot_weight', 'Steering Angle Rate Weight', (0.0, 10.0), 1.0)
    control.add_scalar('phi_dot_max', 'Maximum Value of dPhi/dt', (0.0, 3.0), 1.5)
    control.add_scalar('phi_dot_max_bound_weight',
                       'Weight for Enforcing Maximum Value of dPhi/dt', (0.0, 250.0), 75.0)

    full_menu.generate(
        namespace=("planning", "drifter"),
        cfg_loc="planning/drifter/drifter_configuration",
        ui_loc="planning/drifter/drifter_ui_elements"
    )


if __name__ == '__main__':
    main()
