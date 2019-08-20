import generate_opencl_structs


def main():
    cfg_defd = [
        {
            'type': 'float',
            'length': 1,
            'name': 'nu',
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'dt',
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'dx',
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
        }
    ]

    definitions = [
        ("FluidSimConfig", cfg_defd),
    ]
    destination = "/home/jacob/repos/experiments/gpgpu/kernels/fluid_types"
    generate_opencl_structs.write_files(definitions, destination)


if __name__ == '__main__':
    main()
