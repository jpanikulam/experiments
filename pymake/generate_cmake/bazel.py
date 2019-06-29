import os
from functools import partial

from generate_cmake.log import Log
from generate_cmake.file_system import reduce_srcs


def should_create_target(build_item):
    if 'NOPYMAKE' in os.listdir(build_item['location']):
        Log.debug("Ignoring: {}".format(build_item['target']))
        return False
    else:
        return True


def create_lib(build_item, base_directory=""):
    bloated_srcs = map(partial(os.path.join, build_item['location']), build_item['srcs'])
    # txt = "add_library({} {})".format(build_item['target'], " ".join(reduced_srcs))

    if(len(build_item['deps'])):
        import IPython; IPython.embed(); exit(0)

    txt = """
cc_library(
name = {name},
srcs = [{sources}],
hdrs = [{headers}],
deps = [{deps}]
visibility = ["//visibility:public"]
)
""".format(
        name=build_item['target'],
        sources=", ".join(build_item['hdrs']),
        headers=", ".join(build_item['hdrs'])
        # deps=""
    )

    # If the library has dependencies
    if len(build_item['deps']):
        txt += "\ntarget_link_libraries({} {})".format(build_item['target'], " ".join(build_item['deps']))
    return txt


def create_bin(build_item, base_directory=""):
    reduced_srcs = reduce_srcs(build_item['srcs'], base_directory)

    if 'is_test' in build_item.get('flags', []):
        assert len(reduced_srcs) == 1, "A test only supports one source file"
        txt = 'add_test({target} {srcs} {libs})'.format(
            target=build_item['target'],
            srcs=reduced_srcs[0],
            libs=" ".join(build_item['deps']) if len(build_item['deps']) else '""'
        )
    else:
        txt = "add_executable({} {})".format(build_item['target'], " ".join(reduced_srcs))
        if len(build_item['deps']):
            txt += "\ntarget_link_libraries({} {})".format(build_item['target'], " ".join(build_item['deps']))
    return txt


def build_bazel_build(to_build, base_directory):
    actions = {
        'lib': create_lib,
        'binary': create_bin
    }

    from graph import dependency_sort
    write_order = dependency_sort(to_build)

    Log.success('\n\nGenerating BUILDs....')
    cmake_text = ""
    for write in write_order:
        if write in to_build:
            build_item = to_build[write]
            if not should_create_target(build_item):
                continue

            action = actions[build_item['kind']]
            new_text = action(build_item, base_directory=base_directory)
            cmake_text += new_text + '\n'
            Log.info(new_text)

    # write_build(cmake_text, base_directory)
