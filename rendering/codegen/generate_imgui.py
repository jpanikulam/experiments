from collections import OrderedDict


def line(txt, depth=0):
    return ("  " * depth) + txt + ";\n"


def create_ui_element(submenu, entity):
    kwargs = {
        'readable_name': entity['edit_name'],
        'name': entity['name'],
        'submenu': submenu,
        'default': entity['default'],
    }

    if 'range' in entity:
        kwargs['min'] = entity['range'][0]
        kwargs['max'] = entity['range'][1]

    if "possible_values" in entity:
        kwargs["count"] = len(entity['possible_values'])
        kwargs["possible_values"] = ', '.join('"{0}"'.format(w) for w in entity['possible_values'])

    txt = "{"
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

    elif entity['type'] == 'selector':
        txt += line('const char* element_names[{count}] = {{{possible_values}}}', 2).format(
            **kwargs
        )

        txt += line('const char* current_element_name = ' +
                    '(menu->{submenu}.{name} >= 0 && menu->{submenu}.{name} < {count}) ' +
                    '? element_names[menu->{submenu}.{name}] : "Unknown"', 2).format(**kwargs)

        # txt += 'if (ImGui::Button("Reset##{readable_name}")) {{\n'.format(**kwargs)
        # txt += '  menu->{submenu}.{name} = {default};\n'.format(**kwargs)
        # txt += '}\n'
        # txt += 'ImGui::SameLine();\n'
        txt += line('ImGui::SliderInt("{readable_name}", &(menu->{submenu}.{name}), 0, {count} - 1, current_element_name)'.format(
            **kwargs
        ), 2)

    txt += "}"
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
            if vv['type'] not in ('selector',):
                txt += line("{type} {name} = {default}".format(**vv), 1)
            else:
                default_val = vv['possible_values'].index(vv['default'])
                txt += line("int {name} = {default_val}".format(default_val=default_val, **vv), 1)

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

    def add_scalar(self, name, edit_name, value_range, default=None):
        if default is None:
            default = value_range[0]
        self.submenu[name] = {
            'name': name,
            'edit_name': edit_name,
            'type': 'double',
            'default': default,
            'range': value_range
        }

    def add_int_scalar(self, name, edit_name, value_range, default):
        self.submenu[name] = {
            'name': name,
            'edit_name': edit_name,
            'type': 'int',
            'default': default,
            'range': value_range
        }

    def add_selector(self, name, edit_name, possible_values, default):
        self.submenu[name] = {
            'name': name,
            'edit_name': edit_name,
            'type': 'selector',
            'default': default,
            'possible_values': possible_values
        }


class MenuGenerator(object):
    def __init__(self, name):
        self.full_menu = OrderedDict()
        self.name = name

    def add_submenu(self, type_name, edit_name=None):
        if (edit_name is None):
            edit_name = type_name

        self.full_menu[type_name] = SubMenuGenerator(type_name, edit_name)
        return self.full_menu[type_name]

    def generate(self, namespace, cfg_loc, ui_loc):
        generable_menu = OrderedDict()

        for subname, submnu in self.full_menu.items():
            generable_menu[subname] = submnu.submenu

        hdr_deps = ["<cstddef>"]
        struct_txt = generate_struct(self.name, generable_menu)
        form_header(cfg_loc, namespace, hdr_deps, struct_txt)

        src_deps = [
            '"planning/simulation/sim_viewer_palette.hh"',
            '"third_party/imgui/imgui.h"'
        ]
        menu_txt = generate_menu(self.name, generable_menu)
        form_src(ui_loc, namespace, src_deps, menu_txt)
