import os
import re

from parse import between

valid_extensions = ['cc', 'hh']


def parse_cfg(path, base):
    with open(os.path.join(path, 'cfg.cppmv')) as f:
        text = f.read()

    include_paths = []
    commands = text.split('\n')
    for command in commands:
        if(command.startswith('add_path') or
           command.startswith('add_ext_path')):
            include_paths.append(
                os.path.join(
                    base,
                    between(command, '(', ')')
                )
            )

    return {'include_paths': include_paths}


def resolve_include_TODO(file_path, include_path, available_include_paths):
    dirname = os.path.dirname(file_path)
    other_files = filter(os.path.isfile, os.listdir(dirname))

    if include_path in other_files:
        return os.path.join(dirname, include_path)
    # TODO: Make this handle more cases


def resolve_include(file_path, include_path, available_include_paths=None):
    return os.path.join('/home/jacob/repos/experiments', include_path)


def parse_ignore(path):
    print path
    assert '.ignore' in os.listdir(path)
    with open(os.path.join(path, '.ignore')) as f:
        text = f.read()
    ignores = text.split('\n')
    return list(map(re.compile, ignores))


def deep_listdir(path):
    dir_contents = os.listdir(path)
    return map(lambda o: os.path.join(path, o), dir_contents)


def get_files(path, ignores_path=None, ignores=None):
    from functools import partial
    if ignores is None:
        ignores = parse_ignore(ignores_path)

    def want_file(file):
        re_partial = partial(re.match, string=file)
        allow_by_is_file = os.path.isfile(file)
        allow_by_extension = any(map(file.endswith, valid_extensions))
        allow_by_ignores = not any(map(re_partial, ignores))

        requires = [
            allow_by_ignores,
            allow_by_extension,
            allow_by_is_file
        ]

        return all(requires)

    def want_dir(dirname):
        _, final_dirname = os.path.split(dirname)
        re_partial = partial(re.match, string=final_dirname + '/')
        allow_by_ignores = not any(map(re_partial, ignores))
        return allow_by_ignores and os.path.isdir(dirname)

    dir_elements = deep_listdir(path)
    files_here = filter(want_file, dir_elements)
    dirs_here = filter(want_dir, dir_elements)

    for subdir in dirs_here:
        subdir_path = os.path.join(path, subdir)
        files_there = get_files(subdir_path, ignores=ignores)
        files_here.extend(files_there)
    return files_here
