struct __attribute__((packed)) ProjectionCoefficients {
  float fx;
  float fy;
  float cx;
  float cy;

  float p1;
  float p2;

  float k1;
  float k2;
  float k3;

  int rows;
  int cols;
};

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
    __read_only image2d_t ray_lut0,
    __read_only image2d_t ray_lut1,
    __read_only image2d_t ray_lut2,
    __write_only image2d_t output_image,
    const sampler_t sampler,
    const float theta) {
    struct clSE3 camera_from_plane;
    camera_from_plane.rotation.r0.x = cos(theta);
    camera_from_plane.rotation.r0.y = -sin(theta);
    camera_from_plane.rotation.r0.z = 0.0;
    camera_from_plane.rotation.r1.x = sin(theta);
    camera_from_plane.rotation.r1.y = cos(theta);
    camera_from_plane.rotation.r1.z = 0.0;
    camera_from_plane.rotation.r2.x = 0.0;
    camera_from_plane.rotation.r2.y = 0.0;
    camera_from_plane.rotation.r2.z = 1.0;
    // camera_from_plane.rotation = camera_from_plane_rot;

    camera_from_plane.translation.x = -0.5;
    camera_from_plane.translation.y = -0.5;
    camera_from_plane.translation.z = 3.0;

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

    // const float3 ray_dir_camera_frame = read_imagef(ray_lut, sampler, write_coords).xyz;

    const int4 coord1 = (int4) (col, row, 0, 0);
    const int4 coord2 = (int4) (col, row, 1, 0);
    const int4 coord3 = (int4) (col, row, 2, 0);
    const float3 ray_dir_camera_frame = (float3) (
        read_imagef(ray_lut0, write_coords).x,
        read_imagef(ray_lut1, write_coords).x,
        read_imagef(ray_lut2, write_coords).x
    );
    const float3 ray_origin_camera_frame = (float3) (0.0, 0.0, 0.0);
    const float3 intersection_camera_frame = raycast(
        plane_normal_camera_frame,
        plane_origin_camera_frame,
        ray_dir_camera_frame,
        ray_origin_camera_frame
    );

    const struct clSE3 plane_from_camera = inverse_se3(camera_from_plane);
    const float2 intersection_plane_frame = mul_se3(plane_from_camera, intersection_camera_frame).xy;

    // const float2 coords_input_frame = mul_se2(camera_from_plane, sample_coords);
    // const float3 coords_input_frame
    // const float4 pixel = read_imagef(input_image, sampler, coords_input_frame);

    const float4 pixel = read_imagef(input_image, sampler, intersection_plane_frame * 100.0f);
    write_imagef(output_image, write_coords, pixel.x);

    // if(theta < 1.0) {
    //     write_imagef(output_image, write_coords, ray_dir_camera_frame.x * 255.0);
    // } else if((theta > 1.0) && (theta < 2.0)) {
    //     write_imagef(output_image, write_coords, ray_dir_camera_frame.y * 255.0);
    // } else {
    //     write_imagef(output_image, write_coords, ray_dir_camera_frame.z * 255.0);
    // }
}
