from collections import OrderedDict


def line(txt, depth=0):
    return ("  " * depth) + txt + ";\n"


def create_ui_element(submenu, entity):
    kwargs = {
        'readable_name': entity['edit_name'],
        'name': entity['name'],
        'min': entity['range'][0],
        'max': entity['range'][1],
        'submenu': submenu,
        'default': entity['default']
    }

    txt = ""
    if entity['type'] == 'double':
        txt += 'if (ImGui::Button("Reset##{readable_name}")) {{\n'.format(**kwargs)
        txt += '  menu->{submenu}.{name} = {default};\n'.format(**kwargs)
        txt += '}\n'
        txt += 'ImGui::SameLine();\n'
        txt += line("float {name}_tmp = static_cast<float>((menu->{submenu}.{name}))".format(**kwargs), 2)
        txt += line('ImGui::SliderFloat("{readable_name}", &{name}_tmp, {min}, {max})'.format(
            **kwargs
        ), 2)

        txt += line("menu->{submenu}.{name} = static_cast<double>({name}_tmp)".format(**kwargs), 2)

    elif entity['type'] == 'int':
        # txt += line("float {name}_tmp = static_cast<float>((menu->{submenu}.{name}))".format(**kwargs), 2)
        txt += 'if (ImGui::Button("Reset##{readable_name}")) {{\n'.format(**kwargs)
        txt += '  menu->{submenu}.{name} = {default};\n'.format(**kwargs)
        txt += '}\n'
        txt += 'ImGui::SameLine();\n'
        txt += line('ImGui::SliderInt("{readable_name}", &(menu->{submenu}.{name}), {min}, {max})'.format(
            **kwargs
        ), 2)

    elif entity['type'] == 'range':
        # TODO
        txt = line(
            'ImGui::DragFloatRange2(' +
            '"{readable_name}", &begin, &end, 0.25f, 0.0f, 100.0f, "Min: %.1f", "Max: %.1f"' +
            ')'
        )

    elif entity['type'] == 'bool':
        txt += line('ImGui::Checkbox("{readable_name}", &menu->{submenu}.{name})'.format(
            **kwargs
        ), 2)
        pass

    elif entity['type'] == 'vector2':
        pass

    return txt


def generate_submenu(name, submenu):
    # ImGuiTreeNodeFlags_DefaultOpen
    txt = """
  if (ImGui::CollapsingHeader("{menu_name}")) {{
    if (ImGui::Button("Reset {menu_name}")) {{
      menu->{menu_name_lwr} = {menu_name}{{}};
    }}
{txt}  }}\n"""

    values = ""
    for k, v in submenu.items():
        values += create_ui_element(name.lower(), v)

    return txt.format(menu_name=name, txt=values, menu_name_lwr=name.lower())


def generate_menu(struct_name, full_menu):
    txt = "void show_menu(Out<{struct_name}> menu) {{".format(struct_name=struct_name)
    txt += line('ImGui::Begin("{struct_name}")', 1).format(struct_name=struct_name)
    txt += '  if (ImGui::Button("Reset All")) {\n'
    txt += '    *menu = {struct_name}{{}};'.format(struct_name=struct_name)
    txt += '  }'

    for k, v in full_menu.items():
        txt += generate_submenu(k, v)

    txt += line("ImGui::End()", 1)
    txt += "}"

    return txt


def generate_struct(name, full_menu):
    txt = ""
    for k, submenu in full_menu.items():
        txt += "struct {name} {{\n".format(name=k)

        for kk, vv in submenu.items():
            txt += line("{type} {name} = {default}".format(**vv), 1)

        txt += line("}")

    txt += "struct {name} {{\n".format(name=name)
    for submenu_name, submenu in full_menu.items():
        txt += line("{type} {name}".format(type=submenu_name, name=submenu_name.lower()), 1)

    txt += line("}")

    return txt


def form_header(destination, namespaces, deps, text):
    txt = "#pragma once\n"

    for dep in deps:
        txt += '#include {}\n'.format(dep)

    for namespace in namespaces:
        txt += "namespace {} {{\n".format(namespace)

    txt += text

    for namespace in namespaces[::-1]:
        txt += "}} // namespace {}\n".format(namespace)

    with open("/home/jacob/repos/experiments/" + destination + '.hh', 'w') as f:
        f.write(txt)

    return txt


def form_src(destination, namespaces, deps, text):
    txt = '#include "{destination}.hh"\n'.format(destination=destination)
    for dep in deps:
        txt += '#include {}\n'.format(dep)

    for namespace in namespaces:
        txt += "namespace {} {{\n".format(namespace)

    txt += text

    for namespace in namespaces[::-1]:
        txt += "}} // namespace {}\n".format(namespace)

    with open("/home/jacob/repos/experiments/" + destination + '.cc', 'w') as f:
        f.write(txt)

    return txt


class SubMenuGenerator(object):
    def __init__(self, name, edit_name):
        self.submenu = OrderedDict()
        self.edit_name = edit_name

    def add_toggle(self, name, edit_name, default='false'):
        self.submenu[name] = {
            'name': name,
            'edit_name': edit_name,
            'type': 'bool',
            'default': default,
            'range': (False, True)
        }

    def add_scalar(self, name, edit_name, default, value_range):
        self.submenu[name] = {
            'name': name,
            'edit_name': edit_name,
            'type': 'double',
            'default': default,
            'range': value_range
        }


class MenuGenerator(object):
    def __init__(self, name):
        self.full_menu = OrderedDict()
        self.name = name

    def add_submenu(self, type_name, edit_name):
        self.full_menu[type_name] = SubMenuGenerator(type_name, edit_name)
        return self.full_menu[type_name]


def main():
    # full_menu = OrderedDict()

    #
    ############
    #

    full_menu = MenuGenerator("PlannerConfiguration")

    full_menu["Debug"] = OrderedDict()
    full_menu["Debug"]["visualize_last_state"] = {
        'name': 'visualize_last_state',
        'edit_name': 'Visualize Last State Cost',
        'type': 'bool',
        'default': "false",
        'range': (False, True)
    }
    full_menu["Debug"]["show_cost"] = {
        'name': 'show_cost',
        'edit_name': 'Show Cost',
        'type': 'bool',
        'default': "true",
        'range': (False, True)
    }
    full_menu["Debug"]["enable_look_target"] = {
        'name': 'enable_look_target',
        'edit_name': 'Enable Look Target',
        'type': 'bool',
        'default': "false",
        'range': (False, True)
    }

    full_menu["Debug"]["show_trajectory"] = {
        'name': 'show_trajectory',
        'edit_name': 'Show Trajectory',
        'type': 'bool',
        'default': "true",
        'range': (False, True)
    }

    full_menu["Debug"]["convenience_flag"] = {
        'name': 'convenience_flag',
        'edit_name': 'Flip the Convenience Flag',
        'type': 'bool',
        'default': "true",
        'range': (False, True)
    }

    #
    ############
    #

    full_menu["Optimization"] = OrderedDict()
    full_menu["Optimization"]["max_iters"] = {
        'name': 'max_iterations',
        'edit_name': 'Max Iterations',
        'type': 'int',
        'default': "15",
        'range': (0, 100)
    }
    full_menu["Optimization"]["min_state_damping"] = {
        'name': 'min_state_damping',
        'edit_name': 'Minimum State Damping',
        'type': 'double',
        'default': "1.0",
        'range': (1e-5, 10.0)
    }
    full_menu["Optimization"]["min_ctrl_damping"] = {
        'name': 'min_ctrl_damping',
        'edit_name': 'Minimum Control Damping',
        'type': 'double',
        'default': "1.0",
        'range': (1e-5, 10.0)
    }

    full_menu["Optimization"]["qxx_min_eigenvalue"] = {
        'name': 'qxx_min_eigenvalue',
        'edit_name': 'State Minimum Eigenvalue',
        'type': 'double',
        'default': "1.0",
        'range': (1e-5, 1.0)
    }
    full_menu["Optimization"]["quu_min_eigenvalue"] = {
        'name': 'quu_min_eigenvalue',
        'edit_name': 'Control Minimum Eigenvalue',
        'type': 'double',
        'default': "1.0",
        'range': (1e-5, 1.0)
    }

    #
    ############
    #

    full_menu["Bounds"] = OrderedDict()
    full_menu["Bounds"]["speed_bound_weight"] = {
        'name': 'v_max_weight',
        'edit_name': 'Speed Bound Weight',
        'type': 'double',
        'range': (10.0, 500.0),
        'default': 250.0
    }

    full_menu["Bounds"]["max_speed"] = {
        'name': 'max_speed',
        'edit_name': 'Maximum Speed (m/s)',
        'type': 'double',
        'range': (0.0, 15.0),
        'default': 15.0
    }

    full_menu["Bounds"]["min_speed"] = {
        'name': 'min_speed',
        'edit_name': 'Minimum Speed (m/s)',
        'type': 'double',
        'range': (-10.0, 0.0),
        'default': -10.0
    }

    full_menu["Bounds"]["max_accel"] = {
        'name': 'max_accel',
        'edit_name': 'Maximum Acceleration (m/s^2)',
        'type': 'double',
        'range': (0.0, 1.0),
        'default': 0.5
    }

    #
    ############
    #

    full_menu["Bounds"]["phi_weight"] = {
        'name': 'phi_weight',
        'type': 'double',
        'edit_name': 'Steering Angle Weight',
        'range': (0.0, 25.0),
        'default': 0.1,
    }

    full_menu["Bounds"]["phi_max"] = {
        'name': 'phi_max',
        'type': 'double',
        'edit_name': 'Maximum Value of Phi',
        'range': (0.0, 5.0),
        'default': 3.0,
    }

    full_menu["Bounds"]["phi_max_bound_weight"] = {
        'name': 'phi_max_bound_weight',
        'type': 'double',
        'edit_name': 'Weight for Enforcing Maximum Value of Phi',
        'range': (0.0, 250.0),
        'default': 75.0,
    }

    #
    ############
    #

    full_menu["Goal"] = OrderedDict()
    full_menu["Goal"]["goal_weight"] = {
        'name': 'goal_weight',
        'type': 'double',
        'edit_name': 'Goal Weight',
        'range': (0.0, 10.0),
        'default': 1.0
    }

    full_menu["Goal"]["terminal_vel_weight"] = {
        'name': 'terminal_vel_weight',
        'type': 'double',
        'edit_name': 'Terminal Velocity Weight',
        'range': (0.0, 20.0),
        'default': 10.0
    }

    full_menu["Goal"]["pointing_weight"] = {
        'name': 'pointing_weight',
        'type': 'double',
        'edit_name': 'Pointing Target Weight',
        'range': (0.0, 20.0),
        'default': 4.0
    }

    full_menu["Goal"]["path_weight"] = {
        'name': 'path_weight',
        'type': 'double',
        'edit_name': 'Path Weight',
        'range': (0.0, 20.0),
        'default': 1.0
    }
    full_menu["Goal"]["path_margin"] = {
        'name': 'path_margin',
        'type': 'double',
        'edit_name': 'Path Margin (m)',
        'range': (0.0, 1.0),
        'default': 0.2
    }
    full_menu["Goal"]["enable_path"] = {
        'name': 'enable_path',
        'type': 'bool',
        'edit_name': 'Enable Path',
        'range': (False, True),
        'default': "false"
    }

    #
    ############
    #

    full_menu["Obstacles"] = OrderedDict()
    full_menu["Obstacles"]["avoid_weight"] = {
        'name': 'avoid_weight',
        'type': 'double',
        'edit_name': 'Avoid Area Weight',
        'range': (0.0, 10.0),
        'default': 3.0
    }

    #
    ############
    #

    full_menu["Control"] = OrderedDict()
    full_menu["Control"]["acceleration_weight"] = {
        'name': 'acceleration_weight',
        'type': 'double',
        'edit_name': 'Acceleration Weight',
        'range': (0.0, 2.0),
        'default': 0.1
    }

    full_menu["Control"]["phidot_weight"] = {
        'name': 'phidot_weight',
        'type': 'double',
        'edit_name': 'Steering Angle Rate Weight',
        'range': (0.0, 10.0),
        'default': 1.0,
    }

    full_menu["Control"]["phi_dot_max"] = {
        'name': 'phi_dot_max',
        'type': 'double',
        'edit_name': 'Maximum Value of dPhi/dt',
        'range': (0.0, 3.0),
        'default': 1.5,
    }

    full_menu["Control"]["phi_dot_max_bound_weight"] = {
        'name': 'phi_dot_max_bound_weight',
        'type': 'double',
        'edit_name': 'Weight for Enforcing Maximum Value of dPhi/dt',
        'range': (0.0, 250.0),
        'default': 75.0,
    }

    namespace = ("planning", "drifter")

    hdr_deps = ["<cstddef>"]
    cfg_loc = "planning/drifter/drifter_configuration"
    struct_txt = generate_struct("PlannerConfiguration", full_menu)
    form_header(cfg_loc, namespace, hdr_deps, struct_txt)

    src_deps = [
        '"planning/simulation/sim_viewer_palette.hh"',
        '"third_party/imgui/imgui.h"'
    ]
    ui_loc = "planning/drifter/drifter_ui_elements"
    menu_txt = generate_menu("PlannerConfiguration", full_menu)
    form_src(ui_loc, namespace, src_deps, menu_txt)

if __name__ == '__main__':
    main()
