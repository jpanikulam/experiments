#include "std_helpers.clh"
#include "fluid_defs.clh"
#include "solve_poisson.clh"

//
// Symbol table
// ·∇, ∇²
//

//
// du
// ── = -(u·∇)u - (1/ϱ)∇p + ν·∇²u(x, t) + F(t)
// dt
//
// w = -(u·∇)u + ν·∇²u(x, t) + F(t)
//
// w - (1/ϱ)∇p = du/dt
//

struct VoxelGridDescription {
    float3 origin_m;
    float spacing_m;
};


__kernel void apply_velocity_bdry_cond(
    __read_only image3d_t u0,
    __write_only image3d_t u1) {

    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    // This is terrible, I know. We'll make it faster once it's stable.
    const sampler_t smp_volume_nearest =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP |
        CLK_FILTER_NEAREST;

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);
    const float3 u_inner_bdry = read_imagef(u0, smp_volume_nearest, vxl_coord).xyz;
    if (vxl_coord.x == 99) {
        write_imagef(u1, vxl_coord + unit_x, (float4) (-u_inner_bdry, 0.0f));
    }
    if (vxl_coord.x == 1) {
        write_imagef(u1, vxl_coord - unit_x, (float4) (-u_inner_bdry, 0.0f));
    }
    if (vxl_coord.y == 99) {
        write_imagef(u1, vxl_coord + unit_y, (float4) (-u_inner_bdry, 0.0f));
    }
    if (vxl_coord.y == 1) {
        write_imagef(u1, vxl_coord - unit_y, (float4) (-u_inner_bdry, 0.0f));
    }
    if (vxl_coord.z == 99) {
        write_imagef(u1, vxl_coord + unit_z, (float4) (-u_inner_bdry, 0.0f));
    }
    if (vxl_coord.z == 1) {
        write_imagef(u1, vxl_coord - unit_z, (float4) (-u_inner_bdry, 0.0f));
    }
    write_imagef(u1, vxl_coord, (float4) (u_inner_bdry, 0.0f));
}

__kernel void advect_velocity(
    __read_only image3d_t vol_u_0,
    __write_only image3d_t vol_u_1,
    struct FluidSimConfig cfg) {
    //
    // u0 = vol_u_0[x]
    // vol_u_1[x] = vol_u_0[x - (u0 * dt_sec)]
    //
    const sampler_t smp_volume_interp =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP_TO_EDGE |
        CLK_FILTER_LINEAR;

    const sampler_t smp_volume_nearest =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP |
        CLK_FILTER_NEAREST;

    const int4 vxl_coordi = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);
    const float3 u0 = read_imagef(vol_u_0, smp_volume_nearest, vxl_coordi).xyz;

    const float3 vxl_coordf = convert_float4(vxl_coordi).xyz;
    const float3 u0_real_coord = (vxl_coordf / cfg.dx_m);

    const float3 u1_real_coord = u0_real_coord - (u0 * cfg.dt_sec);
    const float3 u1_vxl_coordf = u1_real_coord * cfg.dx_m;
    const int4 u1_vxl_coordi = (int4) (convert_int3(u1_vxl_coordf), 0);
    const float4 u1 = read_imagef(vol_u_0, smp_volume_interp, u1_vxl_coordi);

    float4 force = (float4) (0.0f, 0.0f, 0.0f, 0.0f);
    if ((vxl_coordi.x > 1) &&
        (vxl_coordi.x < 10) &&
        (vxl_coordi.y > 1) &&
        (vxl_coordi.y < 10) &&
        (vxl_coordi.z > 1) &&
        (vxl_coordi.z < 10)) {
        force.x = -10.1;
        force.y = -10.1;
        force.z = -10.1;
    }
    float4 u1_damped = u1 + (force * cfg.dt_sec);

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);

    write_imagef(vol_u_1, vxl_coordi, u1_damped);
}

/*
__kernel void advect_scalar(
    __read_only image3d_t vol_u_0,
    __read_only image3d_t vol_q_0,
    __write_only image3d_t vol_q_1,
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


// Viscous Diffusion
//
// du
// ── = ν·∇²u(x, t) =
// dt

__kernel void diffuse(
    __read_only image3d_t vol_u_0,
    __read_only image3d_t vol_u_next1,
    __write_only image3d_t vol_u_next2,
    struct FluidSimConfig cfg) {
    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    const float alpha = (cfg.dx_m * cfg.dx_m) / (cfg.nu * cfg.dt_sec);
    const float beta = 4.0f + alpha;

    const float4 jacobi_iterate = poisson_step_3d_vector(
        vol_u_next1,
        vol_u_0,
        vxl_coord,
        alpha,
        beta
    );

    write_imagef(vol_u_next2, vxl_coord, jacobi_iterate);
}

__kernel void compute_divergence(
    __read_only image3d_t vol_u,
    __write_only image3d_t vol_out,
    struct FluidSimConfig cfg) {
    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    const sampler_t smp =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP_TO_EDGE |
        CLK_FILTER_NEAREST;

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);

    // +/- x
    const float xk_nx = read_imagef(vol_u, smp, (vxl_coord - unit_x)).x;
    const float xk_px = read_imagef(vol_u, smp, (vxl_coord + unit_x)).x;

    // +/- y
    const float xk_ny = read_imagef(vol_u, smp, (vxl_coord - unit_y)).y;
    const float xk_py = read_imagef(vol_u, smp, (vxl_coord + unit_y)).y;

    // +/- z
    const float xk_nz = read_imagef(vol_u, smp, (vxl_coord - unit_z)).z;
    const float xk_pz = read_imagef(vol_u, smp, (vxl_coord + unit_z)).z;

    const float inv_2dx = 0.5f / (cfg.dx_m * cfg.dx_m);
    float div_u = (xk_pz - xk_nz) * inv_2dx;
    div_u += (xk_py - xk_ny) * inv_2dx;
    div_u += (xk_px - xk_nx) * inv_2dx;

    float4 div = (float4) (div_u, div_u, div_u, 0.0f);
    write_imagef(vol_out, vxl_coord, (float4) div_u);
}


__kernel void apply_pressure_bdry_cond(
    __read_only image3d_t p0,
    __write_only image3d_t p1) {
    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    const sampler_t smp_volume_nearest =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP |
        CLK_FILTER_NEAREST;

    // This is terrible, I know. We'll make it faster once it's stable.

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);
    const float4 p_inner_bdry = read_imagef(p0, smp_volume_nearest, vxl_coord);
    if (vxl_coord.x == 99) {
        write_imagef(p1, vxl_coord + unit_x, p_inner_bdry);
    }
    if (vxl_coord.x == 1) {
        write_imagef(p1, vxl_coord - unit_x, p_inner_bdry);
    }
    if (vxl_coord.y == 99) {
        write_imagef(p1, vxl_coord + unit_y, p_inner_bdry);
    }
    if (vxl_coord.y == 1) {
        write_imagef(p1, vxl_coord - unit_y, p_inner_bdry);
    }
    if (vxl_coord.z == 99) {
        write_imagef(p1, vxl_coord + unit_z, p_inner_bdry);
    }
    if (vxl_coord.z == 1) {
        write_imagef(p1, vxl_coord - unit_z, p_inner_bdry);
    }
    write_imagef(p1, vxl_coord, p_inner_bdry);
}
// du
// ── = ∇·u(x, t)
// dt
//
// w = (u + ∇p)
// ∇²p = ∇·w
__kernel void compute_pressure(
    __read_only image3d_t div_w,
    __read_only image3d_t vol_p0,
    __write_only image3d_t vol_p1,
    struct FluidSimConfig cfg) {
    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    const float alpha = -(cfg.dx_m * cfg.dx_m);
    const float beta = 4.0f;

    const float jacobi_iterate = poisson_step_3d_scalar(
        vol_p0,
        div_w,
        vxl_coord,
        alpha,
        beta
    );
    write_imagef(vol_p1, vxl_coord, (float4) jacobi_iterate);
}

// Eliminate divergence
__kernel void chiron_projection(
    __read_only image3d_t u_divergent,
    __read_only image3d_t vol_p,
    __write_only image3d_t vol_u_1,
    struct FluidSimConfig cfg) {
    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);

    const sampler_t smp =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP_TO_EDGE |
        CLK_FILTER_NEAREST;

    const float3 w = read_imagef(u_divergent, smp, vxl_coord).xyz;

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);

    // Is this just writing the first element and not all elements?
    const float p0x = read_imagef(vol_p, smp, vxl_coord).x;
    const float3 p000 = (float3) (p0x, p0x, p0x);
    const float p100 = read_imagef(vol_p, smp, vxl_coord + unit_x).x;
    const float p010 = read_imagef(vol_p, smp, vxl_coord + unit_y).x;
    const float p001 = read_imagef(vol_p, smp, vxl_coord + unit_z).x;

    const float3 p_smp = (float3) (p100, p010, p001);
    const float3 dp_dx = (p_smp - p000) / cfg.dx_m;

    const float3 u1 = w - dp_dx;

    write_imagef(vol_u_1, vxl_coord, (float4) (u1, 0.0f));

}