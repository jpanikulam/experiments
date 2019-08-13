#include "liegroups.clh"

inline float3 raycast(const float3 plane_normal, const float3 plane_origin, const float3 ray_direction, const float3 ray_origin) {
  const float d =
      dot((plane_origin - ray_origin), plane_normal) / dot(ray_direction, plane_normal);
  return ray_origin + (d * ray_direction);
}

__kernel void ssd_r(
    __read_only image2d_t image1,
    __read_only image2d_t image2,
    __write_only image2d_t output) {
    const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE |
                          CLK_ADDRESS_CLAMP |
                          CLK_FILTER_NEAREST;
                          // CLK_ADDRESS_NONE : Guarantee we won't leave the image

    const int col = get_global_id(0);
    const int row = get_global_id(1);

    const int2 translation = (int2) (col, row);

    //
    // For a hypothetical translation uv_sample = uv0 + (col, row)
    // Compute an SSD for the whole image
    //

    const int2 im1_size = get_image_dim(image1);
    const int2 im2_size = get_image_dim(image2);

    const int region_size = 50;
    const int half_region_size = region_size / 2;
    int ustart = max(col - half_region_size, 0);
    int uend = min(col + half_region_size, im1_size.x);

    int vstart = max(row - half_region_size, 0);
    int vend = min(row + half_region_size, im1_size.y);

    for (int u1 = ustart; u1 < uend; ++u1) {
        for (int v1 = vstart; v1 < vend; ++v1) {
            const int2 coord = (int2) (u1, v1);
            const float4 px_uv = read_imagef(image1, smp, coord).x;
        }
    }
}

__kernel void sum_squared_diff(
    __read_only image2d_t image_1,
    __read_only image2d_t image_2,
    __global float * local_sums,
    __global float * full_sum
    ) {
    const int2 read_coord = (int2) (get_global_id(0), get_global_id(1));
    const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE |
                          CLK_ADDRESS_CLAMP |
                          CLK_FILTER_NEAREST;

    const float im1_val = read_imagef(image_1, smp, read_coord).x;
    const float im2_val = read_imagef(image_2, smp, read_coord).x;
    const float error = (im1_val - im2_val) / 255.0;
    const float error_sq = error * error;
    const float sum_partial = work_group_reduce_add(error_sq);

    const int write_loc = get_group_id(1);

    const int local_col = get_local_id(0);
    const int local_row = get_local_id(1);
    if (local_row == 0 && local_col == 0) {
      local_sums[write_loc] = sum_partial / ((float) get_local_size(0));
    }

    barrier(CLK_GLOBAL_MEM_FENCE);
    if (get_group_id(1) == 0) {
        if (local_row == 0 && local_col == 0) {
            float total = 0.0;
            for (int k = 0; k < 270; ++k) {
                total += local_sums[k];
            }
            full_sum[0] = total;
        }
    }
}

//
// Read from an image
//

__kernel void render(
    __read_only image2d_t input_image,
    __read_only image2d_t ray_lut,
    __write_only image2d_t output_image,
    const sampler_t sampler,
    const struct clSE3 camera_from_plane) {
    //
    // Render shit up
    //

    const float3 plane_normal_plane_frame = (float3) (0.0, 0.0, 1.0);
    const float3 plane_normal_camera_frame = mul_so3(camera_from_plane.rotation, plane_normal_plane_frame);
    const float3 plane_origin_camera_frame = camera_from_plane.translation;

    const int col = get_global_id(0);
    const int row = get_global_id(1);

    float2 sample_coords = (float) (col, row);
    int2 write_coords = (int2) (col, row);

    const float3 ray_dir_camera_frame = read_imagef(ray_lut, sampler, write_coords).xyz;
    const float3 ray_origin_camera_frame = (float3) (0.0, 0.0, 0.0);
    const float3 intersection_camera_frame = raycast(
        plane_normal_camera_frame,
        plane_origin_camera_frame,
        ray_dir_camera_frame,
        ray_origin_camera_frame
    );

    const struct clSE3 plane_from_camera = inverse_se3(camera_from_plane);
    const float2 intersection_plane_frame = mul_se3(plane_from_camera, intersection_camera_frame).xy;

    const float width = get_image_width(input_image);
    const float4 pixel = read_imagef(input_image, sampler, intersection_plane_frame * width);
    write_imagef(output_image, write_coords, pixel.x);
}
