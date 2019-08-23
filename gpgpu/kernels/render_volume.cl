#include "liegroups.clh"


__kernel void populate_test_volume(
    __write_only image3d_t volume,
    float t) {
    const int3 vxl_coord = (int3) (get_global_id(0), get_global_id(1), get_global_id(2));

    const float3 volume_size = (float3) (
        get_image_width(volume),
        get_image_height(volume),
        get_image_depth(volume)
    );

    float3 vxl_coordf;
    vxl_coordf.x = (float) vxl_coord.x;
    vxl_coordf.y = (float) vxl_coord.y;
    vxl_coordf.z = (float) vxl_coord.z;

    const float3 world_pt = vxl_coordf - (volume_size / 2.0f);

    const int4 write_vxl_coord = (int4) (vxl_coord, 0);

    const float3 rcolor = 25.0f * world_pt / (dot(world_pt, world_pt));
    const float4 color = (float4) (rcolor, 1.0f);

    write_imagef(volume, write_vxl_coord, color);
}

// TODO: Volumetric Billboards
__kernel void render_volume(
    __read_only image2d_t ray_lut,
    __read_only image3d_t volume,
    __write_only image2d_t rendered_image,
    const struct clSE3 world_from_camera,
    const float scaling,
    const float slice_coord) {
    const sampler_t smp_ray = CLK_NORMALIZED_COORDS_FALSE |
                              CLK_ADDRESS_CLAMP |
                              CLK_FILTER_NEAREST;

    const sampler_t smp_volume = CLK_NORMALIZED_COORDS_FALSE |
                                 CLK_ADDRESS_CLAMP |
                                 CLK_FILTER_NEAREST;

    const int2 px_coord = (int2) (get_global_id(0), get_global_id(1));

    const float3 ray_dir_camera = read_imagef(ray_lut, smp_ray, px_coord).xyz;
    const float3 ray_dir_world = mul_so3(world_from_camera.rotation, ray_dir_camera);
    const float3 ray_origin_m = world_from_camera.translation;


    const float3 volume_dims_px = (float3) (
        get_image_width(volume),
        get_image_height(volume),
        get_image_depth(volume)
    );

    const float volume_size_m = 1.0f;
    const float volume_m_per_px = volume_size_m / volume_dims_px.x;

    // const float3 volume_origin_m = -(volume_m_per_px * volume_dims_px) / 2.0f;
    // const float3 volume_origin_m = (-2.0f, -2.0f, -2.0f);
    const float3 volume_origin_m = (float3) (0.0f, 0.0f, 0.0f);

    const float step_size = 0.05f;

    float3 integrated_color = (float3) (0.0f, 0.0f, 0.0f);
    float integrated_value = 0.0f;

    int mode = 1;
    int index = 2;
    float3 slice_diagonal_dir = (float3) (0.0f, 0.0f, 0.0f);

    const float cov = 0.8f;
    if (index == 0) {
       slice_diagonal_dir.x = 1.0f;
    } else if (index == 1) {
       slice_diagonal_dir.y = 1.0f;
    } else if (index == 2) {
       slice_diagonal_dir.z = 1.0f;
    }

    const float normalizer = 1.0f / sqrt(2.0f * M_PI_F * cov);

    // TODO: Consider an exponential step size
    for (float dist_m = 0.0f; dist_m < 10.0f; dist_m += step_size) {
        const float3 sample_pos_m = ray_origin_m + (dist_m * ray_dir_world);

        const float4 sample_pos_vx_frame = (float4) ((sample_pos_m - volume_origin_m) / volume_m_per_px, 1.0f);

        // const float error = dot(sample_pos_vx_frame.xyz, slice_diagonal_dir) - slice_coord;
        // const float mahalanobis_d = -error * error * cov;
        // const float weighting = normalizer * exp(0.5 * mahalanobis_d);

        const float4 image_color = scaling * read_imagef(volume, smp_volume, sample_pos_vx_frame);
        integrated_color += image_color.xyz * step_size * 1.0f;
        // integrated_color = min(integrated_color, error);
    }

    const float color_val = dot(slice_diagonal_dir, integrated_color);

    float3 color;
    if (mode == 0) {
        color = integrated_color;
    } else if (mode == 1) {
        color.x = step(0.0f, color_val) * color_val;
        color.y = 0.0f;
        color.z = -step(0.0f, -color_val) * color_val;
    }

    const float3 power_adjusted_color = pow(clamp(color.xyz, 0.0f, 1.0f), 0.45f);
    const float4 final_color = (float4) (power_adjusted_color, 1.0f);
    write_imagef(rendered_image, px_coord, 255.0f * final_color);
}