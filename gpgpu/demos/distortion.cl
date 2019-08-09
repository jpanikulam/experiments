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

float2 apply_distortion(const struct ProjectionCoefficients proj,
                        const float3 world_point) {
  const float2 distorted_point = world_point.xy / world_point.z;

  const float r2 = dot(distorted_point, distorted_point);
  const float x = distorted_point.x;
  const float y = distorted_point.y;
  const float xy = x * y;

  const float x_tan_offset = (2.0 * proj.p1 * xy) + (proj.p2 * (r2 + (2.0 * x * x)));
  const float y_tan_offset = (2.0 * proj.p2 * xy) + (proj.p1 * (r2 + (2.0 * y * y)));

  const float r4 = r2 * r2;
  const float r6 = r2 * r4;

  const float radial_distortion = (proj.k1 * r2) + (proj.k2 * r4) + (proj.k3 * r6);
  const float x_prime = x * (1.0 + radial_distortion) + x_tan_offset;
  const float y_prime = y * (1.0 + radial_distortion) + y_tan_offset;
  return (float2) (x_prime, y_prime);
}

float2 apply_projection(const struct ProjectionCoefficients proj, const float2 p_prime) {
  float2 projected;
  projected.x = (proj.fx * p_prime.x) + proj.cx;
  projected.y = (proj.fy * p_prime.y) + proj.cy;
  return projected;
}

float2 apply_projection_and_distortion(const struct ProjectionCoefficients proj,
                                       const float3 world_point) {
  const float2 p_prime = apply_distortion(proj, world_point);
  const float2 projected = apply_projection(proj, p_prime);
  return projected;
}