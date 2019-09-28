from generate_imgui import MenuGenerator


def main():
    full_menu = MenuGenerator("GameDebugConfiguration")

    # Debug
    debug = full_menu.add_submenu("Debug")
    debug.add_toggle("use_normals", "Use Normals", default="false")
    debug.add_toggle("use_rsm", "Use Reflective Shadow Maps", default="false")
    debug.add_toggle("wireframe", "Use Wireframe", default="false")
    debug.add_scalar("theta", "Theta", (0.0, 3.14), "1.2")
    debug.add_scalar("d", "Distance", (0.0, 5.0), "1.2")

    debug.add_selector(
        "polygon_mode",
        "Polygon Mode",
        possible_values=("GL_BACK", "GL_FRONT", "GL_FRONT_AND_BACK"),
        default="GL_FRONT"
    )

    full_menu.generate(
        namespace=("jcc",),
        cfg_loc="rendering/imgui_elements/game_debug_configuration",
        ui_loc="rendering/imgui_elements/game_ui_elements"
    )


if __name__ == '__main__':
    main()
