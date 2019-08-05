struct __attribute__((packed)) Star {
  float mass;
  float3 p;
  float3 v;
  float3 f;
};

__kernel void simulate_gravity(
    __global struct Star* in_star,
    __global struct Star* out_star,
    int n_stars) {
    const int idx = get_global_id(0);

    const float mass = in_star[idx].mass;
    const float3 p = in_star[idx].p;
    const float3 v = in_star[idx].v;

    const float dt = 0.1f;
    const float G = 1e-9f;
    const float Q = 1e-9f;

    float3 d2x_dt2 = (float3)(0.0, 0.0, 0.0);
    for (int k = 0; k < n_stars; ++k) {
        const float3 error = in_star[k].p - p;
        const float r2 = dot(error, error);
        const float mul = r2 == 0.0f ? 0.0f : 1.0;
        d2x_dt2 += error * G * (mul * in_star[k].mass) / fmax(r2, 1e-3f);

        const float damping_strength = Q / fmax(r2 * r2, 1e-3f);
        const float3 damping = ((v - in_star[k].v) * damping_strength) / mass;
        d2x_dt2 += damping;
    }

    out_star[idx].mass = mass;
    out_star[idx].p = p + dt * (in_star[idx].v + (0.5f * dt * d2x_dt2));
    out_star[idx].v = v + (dt * d2x_dt2);
    out_star[idx].f = d2x_dt2;

    // out_star[idx].mass = mass;
    // out_star[idx].p = p;
    // out_star[idx].v = v;
}