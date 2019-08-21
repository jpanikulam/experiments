float4 poisson_step_3d(
    __read_only image3d_t vol_x_0,
    __read_only image3d_t vol_b,
    int4 vxl_coord,
    float alpha,
    float beta) {

    const sampler_t smp =
        CLK_NORMALIZED_COORDS_FALSE |
        CLK_ADDRESS_CLAMP_TO_EDGE |
        CLK_FILTER_NEAREST;

    const int4 unit_x = (int4) (1, 0, 0, 0);
    const int4 unit_y = (int4) (0, 1, 0, 0);
    const int4 unit_z = (int4) (0, 0, 1, 0);

    const float3 xk_00 = read_imagef(vol_x_0, smp, vxl_coord).xyz;

    // +/- z
    const float3 xk_nz = read_imagef(vol_x_0, smp, (vxl_coord - unit_z)).xyz;
    const float3 xk_pz = read_imagef(vol_x_0, smp, (vxl_coord + unit_z)).xyz;

    // +/- y
    const float3 xk_ny = read_imagef(vol_x_0, smp, (vxl_coord - unit_y)).xyz;
    const float3 xk_py = read_imagef(vol_x_0, smp, (vxl_coord + unit_y)).xyz;

    // +/- x
    const float3 xk_nx = read_imagef(vol_x_0, smp, (vxl_coord - unit_x)).xyz;
    const float3 xk_px = read_imagef(vol_x_0, smp, (vxl_coord + unit_x)).xyz;

    const float3 b_ij = read_imagef(vol_b, smp, vxl_coord).xyz;

    const float inv_beta = 1.0f / beta;

    // One can view this as either Jacobi iteration, or iterated averaging [1]
    const float3 jacobi_iterate = (
        xk_nz +
        xk_pz +
        xk_ny +
        xk_py +
        xk_nx +
        xk_px +
        (alpha * b_ij)
    ) * inv_beta;

    const float4 y = (float4) (jacobi_iterate, 0.0f);
    return y;
}

//
// Jacobi iteration
// Simultaneously solve 3 separable linear systems
//
// Bibliography
// [1] Introduction to Electrodynamics.
//       ch 3.1.4: Laplace's Equation in Three Dimensions
__kernel solve_poisson_3d(
    __read_only cl::Image3D vol_x_0,
    __read_only cl::Image3D vol_b,
    __write_only cl::Image3D vol_y,
    float alpha,
    float beta) {

    const int4 vxl_coord = (int4) (get_global_id(0), get_global_id(1), get_global_id(2), 0);
    const float4 jacobi_iterate = poisson_step_3d(vol_x_0, vol_b, vxl_coord, alpha, beta);
    write_imagef(vol_y, vxl_coord, y);
}