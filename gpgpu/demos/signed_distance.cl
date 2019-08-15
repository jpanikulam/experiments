#include "signed_distance_shapes.clh"

float sd_sphere(const struct Sphere sphere, float3 p) {
    return length(p - sphere.origin) - sphere.r;
}

float sd_plane(const struct Plane plane, float3 p) {
    return dot(p, plane.normal) - plane.d;
}

float subtract_sd(float d1, float d2) {
    return max(-d1, d2);
}

float union_sd(float d1, float d2) {
    return min(d1, d2);
}

float intersect_sd(float d1, float d2) {
    return max(d1, d2);
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

    struct Plane plane;
    plane.normal = (float3) (0.0f, 0.0f, -1.0f);
    plane.d = -5.0;

    const float d_sph1 = sd_sphere(sph1, p);
    const float d_sph2 = sd_sphere(sph2, p);
    const float d_spheres = intersect_sd(d_sph1, d_sph2);

    // const float plane_bgn = sd_plane(plane, p);
    // const float dist = union_sd(d_spheres, plane_bgn);

    return d_spheres;
}

__kernel void compute_sdf(
    __read_only image2d_t ray_lut,
    __write_only image2d_t rendered_image,
    const struct RenderConfig render_cfg,
    const float t) {
    const int2 px_coord = (int2) (get_global_id(0), get_global_id(1));
    const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE |
                          CLK_ADDRESS_CLAMP |
                          CLK_FILTER_NEAREST;

    const float3 ray = read_imagef(ray_lut, smp, px_coord).xyz;

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


    // const float fx = scene(test_pt);
    // float3 d2d_dx2 = (float3) (
    //     ((scene(test_pt + dx, t) - (2.0 * fx) + scene(test_pt - dx, t)) / (eps * eps);
    //     ((scene(test_pt + dx, t) - (2.0 * fx) + scene(test_pt - dx, t)) / (eps * eps);
    //     (scene(test_pt + dx, t) - scene(test_pt - dx, t)) / (2.0f * eps);
    // );
    // float3 d;
    // float3 d;


    int i = 0;
    for (;i < render_cfg.terminal_iteration; ++i) {
        const float3 test_pt = ray * motion;
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

    const float3 point_light = (float3) (0.0, 1.0, 0.0);

    if (render_cfg.debug_mode == 1) {
        color.z = 255.0f * (float)(i) / (float)(render_cfg.terminal_iteration);
    } else if (render_cfg.debug_mode == 2) {
        if (dist_out > 0.0f) {
            color.z = 255.0f * dist_out / CONE_DIST;
        } else {
            color.y = -255.0f * dist_out / CONE_DIST;
        }
    } else if (render_cfg.debug_mode == 3) {
        float bottom = 2.5f;
        float top = 3.5f;
        color.z = 255.0f * (clamp(motion, bottom, top) - bottom) / (top - bottom);
    } else if (render_cfg.debug_mode == 4) {
        color.xyz = 255.0f * normal;
    } else {
        if (hit) {
            const float n_shades = 3.0f;
            if (render_cfg.test_feature) {
                color.x = 255.0 * round(dot(-ray, normalize(normal) * n_shades)) / n_shades;
                color.y = 120.0 * round(dot(-ray, normalize(normal) * n_shades)) / n_shades;
            } else {
                color.x = 255.0 * dot(-ray, normalize(normal));
                color.y = 120.0 * dot(-ray, normalize(normal));
                color.z = 20.0;
            }
            // color.xyz = 255.0f * normal;
        } else {
            float rr = smallest_dist - 0.15;
            // color.y = 0.2 / (rr * rr)
            color.y = 100.0 * exp(-35.0 * (rr * rr));
        }
    }

    write_imagef(rendered_image, px_coord, color);
}