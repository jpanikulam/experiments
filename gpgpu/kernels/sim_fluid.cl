#include "std_helpers.clh"

#include "fluid_types.clh"

//
// Symbol table
// ·∇, ∇²
//

struct FluidSimConfig {
    float dt_sec;
    float dx_m;
    float nu;
};

struct VoxelGridDescription {
    float3 origin_m;
    float spacing_m;
};

__kernel void advect_velocity(
    __read_only cl::Image3D vol_u_0,
    __write_only cl::Image3D vol_u_1,
    struct FluidSimConfig cfg) {
    //
    // u0 = vol_u_0[x]
    // vol_u_1[x] = vol_u_0[x - (u0 * dt_sec)]
    //
    const sampler_t smp_volume = CLK_NORMALIZED_COORDS_FALSE |
                                 CLK_ADDRESS_CLAMP_TO_EDGE |
                                 CLK_FILTER_LINEAR;
    const int4 vxl_coordi = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);
    const float3 u0 = read_imagef(vol_u_0, smp, vxl_coordi).xyz;


    const float3 vxl_coordf = convert_float4(vxl_coordi).xyz;
    const float3 u0_real_coord = (vxl_coordi / cfg.dx_m);

    const float3 write_real_coord = u0_real_coord - (u0 * cfg.dt_sec);
    const float3 write_vxl_coordf = write_real_coord * cfg.dx_m;
    const int4 write_vxl_coordi = (int4) (convert_int3(write_vxl_coordf), 0);

    write_image(vol_u_1, write_vxl_coordi, u_0);
}

/*
__kernel void advect_scalar(
    __read_only cl::Image3D vol_u_0,
    __read_only cl::Image3D vol_q_0,
    __write_only cl::Image3D vol_q_1,
    struct FluidSimConfig cfg) {
    //
    // u0 = vol_u_0[x]
    // vol_u_1[x] = vol_u_0[x - (u0 * dt_sec)]
    //
    const sampler_t smp_volume = CLK_NORMALIZED_COORDS_FALSE |
                                 CLK_ADDRESS_CLAMP_TO_EDGE |
                                 CLK_FILTER_LINEAR;

    const int4 vxl_coordi = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);
    const float3 u0 = read_imagef(vol_u_0, smp, vxl_coordi).xyz;


    const float3 vxl_coordf = convert_float4(vxl_coordi).xyz;
    const float3 u0_real_coord = (vxl_coordi / cfg.dx_m);

    const float3 write_real_coord = u0_real_coord - (u0 * cfg.dt_sec);
    const float3 write_vxl_coordf = write_real_coord * cfg.dx_m;
    const int4 write_vxl_coordi = (int4) (convert_int3(write_vxl_coordf), 0);

    write_image(vol_u_1, write_vxl_coordi, u_0);
}*/


//
//
// du
// ── = -(u·∇)u - (1/ϱ)∇p + ν·∇²u(x, t) + F(t)
// dt
//

// Viscous Diffusion
// ν·∇²u(x, t)
//
// du
// ── = ∇·u(x, t)
// dt
//
// ν·∇²p(x, t) = ∇·u(x, t)
//
__kernel void diffuse(
    __read_only cl::Image3D vol_u_0,
    __write_only cl::Image3D vol_u_1,
    struct FluidSimConfig cfg) {

    const float alpha = cfg.dx_m * cfg.dx_m / cfg.dt_sec
    // const float beta = nu *
}


//
// ν·∇²p(x, t) = ∇·u(x, t)
//
__kernel void compute_pressure(
    __read_only cl::Image3D vol_u_0,
    __write_only cl::Image3D vol_p,
    struct FluidSimConfig cfg) {
}

// Eliminate divergence
__kernel void chiron_projection(
    __read_only cl::Image3D vol_u_0,
    __read_only cl::Image3D vol_p,
    __write_only cl::Image3D vol_u_1,
    struct FluidSimConfig cfg) {
}