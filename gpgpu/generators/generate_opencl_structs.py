

def parse(text):
    for line in text.split('\n'):
        pass


def form_cc_declaration(element):
    types = {
        'vector': 'Eigen::Vector{n}f'.format(n=element['length']),
        'float': 'float',
        'int': 'int',
        'bool': 'bool',
    }

    zeros = {
        'vector': 'Eigen::Vector{n}f::Zero()'.format(n=element['length']),
        'float': '0.0',
        'int': '0',
        'bool': 'false',
    }

    default_value = element.get('default', zeros[element['type']])

    return {
        'type': types[element['type']],
        'name': element['name'], 'default': default_value
    }


def form_openclcc_declaration(element):
    types = {
        'vector': 'cl_float{n}'.format(n=element['length']),
        'float': 'cl_float',
        'int': 'cl_int',
        'bool': 'cl_int',
    }

    return {
        'type': types[element['type']], 'name': element['name']
    }


def form_opencl_declaration(element):
    types = {
        'vector': 'float{n}'.format(n=element['length']),
        'float': 'float',
        'int': 'int',
        'bool': 'int',
    }

    return {
        'type': types[element['type']], 'name': element['name']
    }


def generate_conversion(definition, struct_name):
    func = "  cl{name} convert() const {{\n".format(name=struct_name)

    func += "    cl{name} cvt;\n".format(name=struct_name)
    for element in definition:
        if element['length'] in (3, 4):
            func += "    cvt.{name} = {{{name}.x(), {name}.y(), {name}.z()}};\n".format(**element)
        elif element['type'] == 'bool':
            func += "    cvt.{name} = static_cast<cl_int>({name});\n".format(**element)
        else:
            func += "    cvt.{name} = {name};\n".format(**element)

    func += "    return cvt;\n  }"

    return func


def generate(definition, name):
    cpp = "struct {name} {{\n".format(name=name)

    packed = False

    if (packed):
        openclcc = "struct __attribute__((packed)) cl{name} {{\n".format(name=name)
        opencl = "struct __attribute__((packed)) {name} {{\n".format(name=name)
    else:
        openclcc = "struct cl{name} {{\n".format(name=name)
        opencl = "struct {name} {{\n".format(name=name)
    for element in definition:
        cpp += "  {type} {name} = {default};\n".format(**form_cc_declaration(element))
        openclcc += "  {type} {name};\n".format(**form_openclcc_declaration(element))
        opencl += "  {type} {name};\n".format(**form_opencl_declaration(element))

    cpp += generate_conversion(definition, name)
    cpp += "\n};\n\n"
    openclcc += "};\n\n"
    opencl += "};"

    return {
        'hh': openclcc + cpp,
        'clh': opencl
    }


def write_files(definitions, destination):
    hh = '#pragma once\n\n#include "eigen.hh"\n\n'

    clh = "#pragma once\n"

    for name, definition in definitions:
        genned = generate(definition, name)
        hh += genned['hh']
        clh += '\n' + genned['clh'] + '\n'

    with open(destination + '.clh', 'w') as f:
        f.write(clh)

    with open(destination + '.hh', 'w') as f:
        f.write(hh)
