from generate_imgui import MenuGenerator


def main():
    full_menu = MenuGenerator("GameDebugConfiguration", "Game Debug")

    # Debug
    debug = full_menu.add_submenu("Debug")
    # debug.add_toggle("use_normals", "Use Normals", default="false")

    debug.add_toggle("wireframe", "Use Wireframe", default="false")
    debug.add_scalar("theta", "Theta", (0.0, 3.14), "3.14")
    debug.add_scalar("d", "Distance", (0.0, 5.0), "0.438")

    shading = full_menu.add_submenu("Shading")
    shading.add_toggle("enable_shadows", "Enable shadows", default="true")
    shading.add_toggle("misc_debug", "Enable our mysterious shader debug flag", default="false")
    shading.add_toggle("srgb", "Enable SRGB", default="false")
    shading.add_toggle("use_rsm", "Use Reflective Shadow Maps", default="true")
    shading.add_toggle("show_light_probes", "Show light probe locations in red", default="false")

    full_menu.generate(
        namespace=("jcc",),
        cfg_loc="rendering/imgui_elements/game_debug_configuration",
        ui_loc="rendering/imgui_elements/game_ui_elements"
    )


if __name__ == '__main__':
    main()
