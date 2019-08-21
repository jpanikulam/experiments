# %codegen(cl_gen)
import generate_opencl_structs


def main():
    cfg_defd = [
        {
            'type': 'int',
            'length': 1,
            'name': 'method',
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'step_size',
            'default': '0.01'
        },
        {
            'type': 'float',
            'length': 1,
            'name': 'max_dist',
            'default': "25.0"
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
        ("RenderVolumeConfig", cfg_defd),
    ]
    destination = "/home/jacob/repos/experiments/gpgpu/kernels/render_volume"
    generate_opencl_structs.write_files(definitions, destination)


if __name__ == '__main__':
    main()
