#include "liegroups.clh"
#include "signed_distance_shapes.clh"
#include "signed_distance_operations.clh"

float cel_quantize(float x) {
}


float scene(float3 p, float t) {
    struct Sphere sph1;
    // sph1.origin = (float3) (1.0 * cos(0.1 * t), 1.0 * sin(0.1 * t), 2.5f);
    // sph1.origin = (float3) (0.5 * cos(0.1 * t), 0.0, 3.0f + 0.5 * sin(0.1 * t));
    // sph1.origin = (float3) (0.2, 0.0, 2.5f + (0.5 * sin(0.5 * t)));
    sph1.origin = (float3) (0.2f, 0.0f, 3.5f);
    sph1.r = 1.0f + cos(0.05f * t);

    struct Sphere sph2;
    sph2.origin = (float3) (0.0f, 0.0f, 3.0f);
    sph2.r = 0.5;

    struct Box box;
    // const float3 box_motion = (float3) (0.0, 0.0, 0.5);
    // const float3 box_start = (float3) (0.0, 0.0, 0.0);
    // box.origin = box_motion * 3.0f * (1.0f + cos(0.1f * t)) + 3.0;
    // box.extents = (float3) (0.25f, 0.25f, 3.0f);

    box.origin = (float3) (0.0f, 0.0f, 3.0f);
    box.extents = (float3) (0.25, 0.25, 3.0 * sph1.r * (1.0 + cos(0.1 * t)));

    struct Plane plane;
    plane.normal = normalize((float3) (0.0f, -0.7f, -0.7f));
    plane.d = -5.0;

    const float d_sph1 = sd_sphere(sph1, p);
    const float d_sph2 = sd_sphere(sph2, p);
    const float d_spheres = intersect_sd(d_sph1, d_sph2);

    const float d_box = sd_box(box, p);
    const float d_sphbox = subtract_sd(d_spheres, d_box);

    const float plane_bgn = sd_plane(plane, p);
    const float dist = union_sd(d_sphbox, plane_bgn);

    return dist;
}

float3 illuminate(bool specular, float3 p, float3 view_dir, float3 normal, float3 light_pos, float3 light_color, float t) {
    const float shadow_k = 0.999;
    const float3 r_light = normalize(light_pos - p);
    float shadow_scaling = 1.0;

    //
    // Diffuse
    //

    for(float light_ray_length = 0.1f; light_ray_length < length(light_pos - p);) {
        const float3 light_p = p + (r_light * light_ray_length);
        const float light_d = scene(light_p, t);
        if (fabs(light_d) < 0.001) {
            shadow_scaling = 0.0;
            break;
        }
        light_ray_length += light_d;
        shadow_scaling = min(shadow_scaling, shadow_k * max(light_d, 0.0f) / light_ray_length);
    }

    const float light_dot_normal = dot(r_light, normal);
    // const float3 color = (float3) (
    //     light_color.x * shadow_scaling * light_dot_normal,
    //     light_color.y * shadow_scaling * light_dot_normal,
    //     light_color.z * shadow_scaling * light_dot_normal
    // );
    float3 color = light_color * shadow_scaling * light_dot_normal;

    //
    // Specular
    //

    if (specular) {
        // const float cos_specular = dot(view_dir, reflect(r_light, normal));
        // const float specular_strength = cos_specular > 0.96 ? cos_specular : 0.0;
        // color += light_color * specular_strength * shadow_scaling;

        const float specular_shadow_scaling = step(0.01f, shadow_scaling);

        const float specular_strength = max(0.0f, pow(clamp(dot(r_light, reflect(view_dir, normal)), 0.0f, 1.0f), 1.0f ));
        color += light_color * specular_strength * specular_shadow_scaling;

    }

    return color;
}

__kernel void compute_sdf(
    __read_only image2d_t ray_lut,
    __write_only image2d_t rendered_image,
    const struct clSE3 world_from_camera,
    const struct RenderConfig render_cfg,
    const float zoom,
    const float t) {
    const int2 px_coord = (int2) (get_global_id(0), get_global_id(1));
    const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE |
                          CLK_ADDRESS_CLAMP |
                          CLK_FILTER_NEAREST;

    const float3 ray_camera = read_imagef(ray_lut, smp, px_coord).xyz;

    // const float3 ray = ray_camera;//mul_so3(world_from_camera.rotation, ray_camera);
    const float3 ray = mul_so3(world_from_camera.rotation, ray_camera);
    const float3 ray_origin = world_from_camera.translation;

    float3 normal;
    const float eps = 0.001;
    const float3 dx = (float3) (eps, 0.0, 0.0);
    const float3 dy = (float3) (0.0, eps, 0.0);
    const float3 dz = (float3) (0.0, 0.0, eps);

    const float CONE_DIST = 0.0001;

    float smallest_dist = 1000.0f;
    float motion = 0.1;
    float dist_out = 0.0;
    bool hit = false;

    int i = 0;
    for (;i < render_cfg.terminal_iteration; ++i) {
        const float3 test_pt = ray_origin + (ray * motion);
        dist_out = scene(test_pt, t);

        smallest_dist = min(smallest_dist, dist_out);
        if (fabs(dist_out) < (motion * CONE_DIST)) {
            normal = (float3) (
                (scene(test_pt + dx, t) - dist_out) / eps,
                (scene(test_pt + dy, t) - dist_out) / eps,
                (scene(test_pt + dz, t) - dist_out) / eps
            );
            hit = true;
            break;
        }
        motion += dist_out;
    }

    float4 color = (float4) (0.0f, 0.0f, 0.0f, 255.0f);

    if (render_cfg.debug_mode == 1) {
        color.z = (float)(i) / (float)(render_cfg.terminal_iteration);
    } else if (render_cfg.debug_mode == 2) {
        // Cone distance
        if (dist_out > 0.0f) {
            color.z = dist_out / CONE_DIST;
        } else {
            color.y = -dist_out / CONE_DIST;
        }
    } else if (render_cfg.debug_mode == 3) {
        // Ray length

        // float bottom = 2.5f;
        // float top = 3.5f;
        // color.z = (clamp(motion, bottom, top) - bottom) / (top - bottom);
        color.z = motion / 5.0f;
    } else if (render_cfg.debug_mode == 4) {
        color.xyz = normal;
    } else {
        if (hit) {
            const float n_shades = 3.0f;
            const float3 p = ray_origin + (ray * motion);
            // const float3 r_light = normalize(point_light - p);
            // const float shadow_k = 0.999;
            const float3 light_1_pos = (float3) (2.0 * cos(0.1f * t), 2.0 * cos(0.1 * t), -5.0f);
            const float3 light_1_color = (float3) (0.9f, 0.2f, 0.1f);
            color.xyz += illuminate(true, p, ray, normal, light_1_pos, light_1_color, t);

            const float3 light_2_pos = (float3) (2.0, 2.0, -3.0f);
            const float3 light_2_color = 0.0f * (float3) (0.9f, 0.2f, 0.1f);
            color.xyz += illuminate(true, p, ray, normal, light_2_pos, light_2_color, t);
        } else {
            float rr = smallest_dist - 0.15;
            // color.y = 0.2 / (rr * rr)
            // color.y = 100.0 * exp(-35.0 * (rr * rr));
        }
    }

    // const float3 adjusted_color = 230.0f * pow(color, 0.5);
    // const float4 power_adjusted_color = (float4) (255.0f * tanh(color.xyz), 255.0f);
    const float3 power_adjusted_color = pow(clamp(color.xyz, 0.0f, 3.0f), 0.45f);

    float4 final_color = (float4) (power_adjusted_color, 1.0);
    write_imagef(rendered_image, px_coord, 255.0f * final_color);
}