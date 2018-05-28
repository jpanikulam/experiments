import argparse
import os
from file_system import get_files, resolve_include
from parse import splitstrip, needs, between

pairs = {
    '<': '>',
    '(': ')',
    '"': '"',
    "'": "'",
}


def grab_dependencies(line):
    '''Parse dependencies out of a line'''
    needs(line, ['(', ')'])
    needs(line, ['//%deps'])

    args = between(line, '(', ')')
    deps = splitstrip(args)
    print '    deps: {}'.format(deps)
    return {'deps': deps}


def grab_lib(line):
    needs(line, ['(', ')'])
    needs(line, ['//%lib'])
    args = between(line, '(', ')')
    arg_tokens = splitstrip(args)
    print '    lib: {}'.format(arg_tokens)
    return {'lib': {'target': arg_tokens[0], 'srcs': arg_tokens[1:]}}


def grab_bin(line):
    needs(line, ['(', ')'])
    needs(line, ['//%bin'])
    args = between(line, '(', ')')
    arg_tokens = splitstrip(args)
    print '    bin: {}'.format(arg_tokens)
    assert len(arg_tokens) == 1, "Wrong number of arguments"
    return {'bin': {'target': arg_tokens[0]}}


def grab_include(line):
    needs(line, ['#include'])

    incl_path = ""
    for start, end in pairs.items():
        if start in line:
            incl_path = between(line, start, end)
            break

    if start == '"':
        incl_type = "local"
    elif start == '<':
        incl_type = "system"
    else:
        raise NotImplementedError("wtf?")
    print "    inc: {} ({})".format(incl_path, incl_type)

    return {'include': {'given_path': incl_path, 'type': incl_type}}

tokens = {
    '//%bin': grab_bin,
    '//%deps': grab_dependencies,
    '//%lib': grab_lib,
    '#include': grab_include,
    # '//%hdrlib': grab_hdr_lib,
}


def incremental_update(d, new_d):
    for key, item in new_d.items():
        if key not in d:
            d[key] = []

        if isinstance(item, list):
            d[key].extend(item)
        else:
            d[key].append(item)


def parse_text(text):
    elements = {
        'bin': [],
        'deps': [],
        'lib': [],
        'include': [],
    }

    lines = text.split('\n')
    for line in lines:
        for token, action in tokens.items():
            if line.startswith(token):
                update = action(line)
                incremental_update(elements, update)

    return elements


def create_lib(target_name, srcs, deps):
    reduced_srcs = map(lambda o: os.path.split(o)[-1], srcs)
    txt = "add_library({} {})".format(target_name, " ".join(reduced_srcs))
    if len(deps):
        txt += "\ntarget_link_libraries({} {})".format(target_name, " ".join(deps))
    return txt


def create_bin(target_name, srcs, deps):
    reduced_srcs = map(lambda o: os.path.split(o)[-1], srcs)
    txt = "add_executable({} {})".format(target_name, " ".join(reduced_srcs))
    if len(deps):
        txt += "\ntarget_link_libraries({} {})".format(target_name, " ".join(deps))
    return txt


def get_src_path(file, src):
    fpath = os.path.dirname(file)
    return os.path.join(fpath, src)


def what_libs(file, elements, all_tree, known_libs):
    required_libs = []
    # Obviated dependencies
    for dep in elements['deps']:
        if dep not in known_libs:
            print "  no lib: {}".format(dep)

    required_libs.extend(elements['deps'])

    for lib in elements['lib']:
        for src in lib['srcs']:
            resolved = get_src_path(file, src)
            src_libs = set(what_libs(resolved, all_tree[resolved], all_tree, known_libs))

            # HACK: Don't depend on self
            print lib['target'], src_libs
            src_libs = src_libs.difference({lib['target']})
            required_libs.extend(src_libs)

    # Inferred dependencies
    for include in elements['include']:
        if include['type'] != 'system':
            resolved = resolve_include(file, include['given_path'])
            available = resolved in all_tree.keys()

            if available:
                inferred_libs = all_tree[resolved]['lib']
                for lib in inferred_libs:
                    required_libs.append(lib['target'])

    return required_libs


def build_dependency_table(all_tree):
    libs = []
    for file, elements in all_tree.items():
        for lib in elements['lib']:
            libs.append(lib)

    to_build = {}
    for file, elements in all_tree.items():
        no_deps = len(elements['deps']) == 0
        no_bin = len(elements['bin']) == 0
        no_lib = len(elements['lib']) == 0
        if all([no_deps, no_bin, no_lib]):
            continue

        print file
        required_libs = what_libs(file, elements, all_tree, libs)
        print '  need: {}'.format(required_libs)

        for binary in elements['bin']:
            to_build[binary['target']] = {
                'target': binary['target'],
                'srcs': [file],
                'deps': required_libs,
                'kind': 'binary'
            }

        for lib in elements['lib']:
            to_build[lib['target']] = {
                'target': lib['target'],
                'srcs': lib['srcs'],
                'deps': required_libs,
                'kind': 'lib'
            }

    actions = {
        'lib': create_lib,
        'binary': create_bin
    }

    from graph import dependency_sort
    write_order = dependency_sort(to_build)

    print '\n\n'
    print 'Generating cmake....'
    for write in write_order:
        if write in to_build:
            build_item = to_build[write]
            action = actions[build_item['kind']]
            print action(build_item['target'], build_item['srcs'], build_item['deps'])


def parse_file(path):
    with open(path) as file:
        text = file.read()

    print path
    parse_result = parse_text(text)
    # import json
    # print json.dumps(parse_result, indent=1)
    print '---'
    return {path: parse_result}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--path", type=str)
    args = parser.parse_args()

    here = os.path.dirname(os.path.realpath(__file__))

    if args.path is None:
        path = os.getcwdu()
    else:
        path = args.path

    print "Parsing: ", path
    files = []
    if os.path.isfile(path):
        files.append(path)
    else:
        recursed_files = get_files(path, ignores_path=here)
        files.extend(recursed_files)

    results = {}
    for file in files:
        results.update(parse_file(file))
    build_dependency_table(results)


if __name__ == '__main__':
    main()
