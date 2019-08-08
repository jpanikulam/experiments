struct __attribute__((packed)) clSE3 {
  float3 r0;
  float3 r1;
  float3 r2;

  float3 translation;
};


struct __attribute__((packed)) clSE2 {
    float2 r0;
    float2 r1;
    float2 translation;
};


inline float3 mul_se3(const struct clSE3 m, const float3 v) {
    float3 result;
    result.x = dot(m.r0, v) + m.translation.x;
    result.y = dot(m.r1, v) + m.translation.y;
    result.z = dot(m.r2, v) + m.translation.z;
    return result;
}

inline float2 mul_se2(const struct clSE2 m, const float2 v) {
    float2 result;
    result.x = dot(m.r0, v) + m.translation.x;
    result.y = dot(m.r1, v) + m.translation.y;
    return result;
}

//
// Read from an image
//

__kernel void render(
    __read_only image2d_t input_image,
    __write_only image2d_t output_image,
    const sampler_t sampler,
    const float theta) {
    struct clSE2 input_from_output;
    input_from_output.r0.x = cos(theta);
    input_from_output.r0.y = -sin(theta);
    input_from_output.r1.x = sin(theta);
    input_from_output.r1.y = cos(theta);

    input_from_output.translation.x = 0.0;
    input_from_output.translation.y = 0.0;

    //
    // Render shit up
    //

    const int column = get_global_id(0);
    const int row = get_global_id(1);

    float2 sample_coords;
    sample_coords.x = (float) column;
    sample_coords.y = (float) row;

    const float2 coords_input_frame = mul_se2(input_from_output, sample_coords);
    const float4 pixel = read_imagef(input_image, sampler, coords_input_frame);

    int2 write_coords;
    write_coords.x = column;
    write_coords.y = row;
    write_imagef(output_image, write_coords, pixel.x);
}
