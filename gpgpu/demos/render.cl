struct __attribute__((packed)) clSO3 {
  float3 r0;
  float3 r1;
  float3 r2;
};

struct __attribute__((packed)) clSE3 {
    struct clSO3 rotation;
    float3 translation;
};

inline float3 mul_so3(const struct clSO3 m, const float3 v) {
    float3 result;
    result.x = dot(m.r0, v);
    result.y = dot(m.r1, v);
    result.z = dot(m.r2, v);
    return result;
}

struct __attribute__((packed)) clSE2 {
    float2 r0;
    float2 r1;
    float2 translation;
};

inline struct clSO3 inverse_so3(const struct clSO3 m) {
    struct clSO3 result;
    result.r0.x = m.r0.x;
    result.r0.y = m.r1.x;
    result.r0.z = m.r2.x;

    result.r1.x = m.r0.y;
    result.r1.y = m.r1.y;
    result.r1.z = m.r2.y;

    result.r2.x = m.r0.z;
    result.r2.y = m.r1.z;
    result.r2.z = m.r2.z;
    return result;
}

inline struct clSE3 inverse_se3(const struct clSE3 m) {
    struct clSE3 result;
    result.rotation = inverse_so3(m.rotation);
    result.translation = -mul_so3(result.rotation, m.translation);
    return result;
}

inline float3 mul_se3(const struct clSE3 m, const float3 v) {
    float3 result = mul_so3(m.rotation, v) + m.translation;
    return result;
}

inline float2 mul_se2(const struct clSE2 m, const float2 v) {
    float2 result;
    result.x = dot(m.r0, v) + m.translation.x;
    result.y = dot(m.r1, v) + m.translation.y;
    return result;
}

inline float3 raycast(const float3 plane_normal, const float3 plane_origin, const float3 ray_direction, const float3 ray_origin) {
  const float d =
      dot((plane_origin - ray_origin), plane_normal) / dot(ray_direction, plane_normal);
  return ray_origin + (d * ray_direction);
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

    const float4 pixel = read_imagef(input_image, sampler, intersection_plane_frame * 100.0f);
    write_imagef(output_image, write_coords, pixel.x);
}
