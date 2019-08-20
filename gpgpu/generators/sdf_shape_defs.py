import generate_opencl_structs


def main():
    plane_defd = [
        {
            'type': 'vector',
            'length': 3,
            'name': 'normal',
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'd',
        }
    ]

    sphere_defd = [
        {
            'type': 'vector',
            'length': 3,
            'name': 'origin',
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'r',
        }
    ]

    box_defd = [
        {
            'type': 'vector',
            'length': 3,
            'name': 'origin',
        },
        {
            'type': 'vector',
            'length': 3,
            'name': 'extents',
        },
    ]

    cfg_defd = [
        {
            'type': 'int',
            'length': 1,
            'name': 'debug_mode',
        },
        {
            'type': 'bool',
            'length': 1,
            'name': 'test_feature',
        },
        {
            'type': 'int',
            'length': 1,
            'name': 'terminal_iteration',
        }
    ]

    definitions = [
        ("Plane", plane_defd),
        ("Sphere", sphere_defd),
        ("Box", box_defd),
        ("RenderConfig", cfg_defd),
    ]
    destination = "/home/jacob/repos/experiments/gpgpu/demos/signed_distance_shapes"
    generate_opencl_structs.write_files(definitions, destination)


if __name__ == '__main__':
    main()
