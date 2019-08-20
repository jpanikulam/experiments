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
    // float3 rcolor;
    // rcolor.x = 1.0f / max(world_pt.x, 1e-2f);
    // rcolor.y = world_pt.x * world_pt.y;
    // rcolor.z = world_pt.z * world_pt.z;

    const float4 color = (float4) (rcolor, 1.0f);

    // const float4 color = (float4) (1.0f, 1.0f, 1.0f, 1.0f);
    // const float4 color = (float4) (1.0f, 1.0f, 1.0f, 1.0f);
    write_imagef(volume, write_vxl_coord, color);
}

// TODO: Volumetric Billboards
__kernel void render_volume(
    __read_only image2d_t ray_lut,
    __read_only image3d_t volume,
    __write_only image2d_t rendered_image,
    const struct clSE3 world_from_camera,
    const float t) {
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
    const float3 volume_origin_m = (-0.0f, -0.0f, -0.0f);

    float integrated_value = 0.0f;

    const float step_size = 0.01f;
    const float inv_step_size = 1.0f / step_size;
    float3 integrated_color = (float3) (0.0f, 0.0f, 0.0f);

    for (float dist_m = 0.0f; dist_m < 50.0f; dist_m += step_size) {
        const float3 sample_pos_m = ray_origin_m + (dist_m * ray_dir_world);

        const float4 sample_pos_vx_frame = (float4) ((sample_pos_m - volume_origin_m) / volume_m_per_px, 1.0f);

        const float value = fabs(0.1f * read_imagef(volume, smp_volume, sample_pos_vx_frame).x);
        integrated_color += fabs(read_imagef(volume, smp_volume, sample_pos_vx_frame).xyz);

        // const float value = 0.000001f / (pow(dot(sample_pos_vx_frame, sample_pos_vx_frame), t));
        // const float value = 0.000001f / (dot(sample_pos_vx_frame, sample_pos_vx_frame));
        // const float value = 1.0f;

        // const float max_norm = max(max(fabs(sample_pos_m.x), fabs(sample_pos_m.y)), fabs(sample_pos_m.z));
        // const float incl = step(-2.0f, -max_norm);
        integrated_value += value * step_size;
        // integrated_color = max(integrated_color, incl);
    }

    const float3 color = (float3) (integrated_value, 0.0, 0.0);
    // const float3 color = integrated_color;
    const float3 power_adjusted_color = pow(clamp(color.xyz, 0.0f, 1.0f), 0.45f);
    const float4 final_color = (float4) (power_adjusted_color, 1.0f);
    write_imagef(rendered_image, px_coord, 255.0f * final_color);
}