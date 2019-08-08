struct __attribute__((packed)) Star {
  float mass;
  float3 p;
  float3 v;
  float3 f;
};

inline float compute_grav_force(const float G, const float mass, const float r2) {
    const float mul = r2 == 0.0f ? 0.0f : 1.0;
    return G * (mul * mass) / fmax(r2, 1e-3f);
}

inline float3 compute_damping(const float Q, const float r2, const float3 v, const float3 v_other, const float mass) {
    const float damping_strength = Q / fmax(r2 * r2, 1e-3f);
    return (v - v_other) * damping_strength / mass;
}

__kernel void simulate_gravity(
    __global struct Star* in_star,
    __global struct Star* out_star,
    int n_stars) {
    const int idx = get_global_id(0);

    const float mass = in_star[idx].mass;
    const float3 p = in_star[idx].p;
    const float3 v = in_star[idx].v;

    const float dt = 0.1f;
    const float G = 1e-8f;
    const float Q = 0.0f;

    float3 d2x_dt2 = (float3)(0.0, 0.0, 0.0);
    for (int k = 0; k < n_stars; ++k) {
        const float3 error = in_star[k].p - p;
        const float r2 = dot(error, error);

        d2x_dt2 += error * compute_grav_force(G, mass, r2);
        d2x_dt2 += compute_damping(Q, r2, v, in_star[k].v, mass);
    }

    out_star[idx].mass = mass;
    out_star[idx].p = p + dt * (in_star[idx].v + (0.5f * dt * d2x_dt2));
    out_star[idx].v = v + (dt * d2x_dt2);
    out_star[idx].f = d2x_dt2;
}