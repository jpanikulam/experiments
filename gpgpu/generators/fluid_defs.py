# %codegen(cl_gen)

import generate_opencl_structs


def main():
    cfg_defd = [
        {
            'type': 'float',
            'length': 1,
            'name': 'nu',
            'default': 0.8
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'dt_sec',
            'default': 0.01,
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'dx_m',
            'default': 0.1,
        },
        {
            'type': 'bool',
            'length': 1,
            'name': 'test_feature',
        },
        {
            'type': 'int',
            'length': 1,
            'name': 'max_iteration',
        },
        {
            'type': 'int',
            'length': 1,
            'name': 'debug_mode',
        }
    ]

    definitions = [
        ("FluidSimConfig", cfg_defd),
    ]
    destination = "/home/jacob/repos/experiments/gpgpu/kernels/fluid_defs"
    generate_opencl_structs.write_files(definitions, destination)

if __name__ == '__main__':
    main()
