#include "liegroups.clh"
#include "signed_distance_shapes.clh"
#include "signed_distance_operations.clh"

//
// Ok, second idea:
//

//
// Object[N] which is binary tree
//

enum Operation {
    UNION = 0,
    INTERSECT = 1,
    SUBTRACT = 2
};

enum DistanceField {
    SPHERE = 0,
    PLANE = 1,
    BOX = 2
};

struct Object {
    union {
        Sphere sphere;
        Plane plane;
        Box box;
    } element;
    DistanceField element_id;
    Operation op_with_parent;
    bool l_child;
    bool r_child;
};

struct Scene {
    int n_objects;
    __global Object* objects;
};

float sd_scene(struct Scene objects, const float3 p, float t) {
    float d = 1000.0f;
    // Ok, now let's some BFS

    int level_start = 1;
    int level_index = 0;

    for (int k = 0; k < objects.n_objects) {
        int addr = level_start + level_index - 1;
        obj = objects.objects[addr];

        if (l_child) {

        }

        level_start = level_start << 1;
        level_index = level_index << 1;
    }

    // d = op_union()

    // for (int k = 0; k < objects.n_objects; ++k) {
        // d = op_union(sd_object(objects.objects[k], p), d);
    // }
    return d;
}

float3 illuminate(
    Scene objects,
    float3 p,
    float3 view_dir,
    float3 normal,
    float3 light_pos,
    float3 light_color,
    float t) {
    const float shadow_k = 0.999f;
    const float3 r_light = normalize(light_pos - p);
    float shadow_scaling = 1.0f;

    //
    // Diffuse
    //

    int steps = 0;
    for(float light_ray_length = 0.1f; light_ray_length < length(light_pos - p);) {
        steps++;
        // if(steps > 1) {
        //     break;
        // }
        const float3 light_p = p + (r_light * light_ray_length);
        const float light_d = sd_scene(objects, light_p, t);
        if (fabs(light_d) < 0.01) {
            shadow_scaling = 0.0;
            break;
        }
        light_ray_length += light_d;
        shadow_scaling = min(shadow_scaling, shadow_k * max(light_d, 0.0f) / light_ray_length);
    }

    const float light_dot_normal = dot(r_light, normal);
    float3 color = light_color * shadow_scaling * light_dot_normal;

    //
    // Specular
    //

    const float specular_shadow_scaling = step(0.01f, shadow_scaling);
    const float specular_strength = max(0.0f, pow(clamp(dot(r_light, reflect(view_dir, normal)), 0.0f, 1.0f), 1.0f ));
    color += light_color * specular_strength * specular_shadow_scaling;

    return color;
}

__kernel void compute_sdf(
    __read_only image2d_t ray_lut,
    __write_only image2d_t rendered_image,
    int n_objects,
    __global struct ComplexObject * objects,
    const struct clSE3 world_from_camera,
    const struct RenderConfig render_cfg,
    const float t) {

    struct Scene in_scene;
    in_scene.objects = objects;
    in_scene.n_objects = n_objects;

    const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE |
                          CLK_ADDRESS_CLAMP |
                          CLK_FILTER_NEAREST;
    const int2 px_coord = (int2) (get_global_id(0), get_global_id(1));
    const float3 ray_camera = read_imagef(ray_lut, smp, px_coord).xyz;

    const float3 ray = mul_so3(world_from_camera.rotation, ray_camera);
    // ?????
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
        dist_out = sd_scene(in_scene, test_pt, t);

        smallest_dist = min(smallest_dist, dist_out);
        if (fabs(dist_out) < (motion * CONE_DIST)) {
            normal = (float3) (
                (sd_scene(in_scene, test_pt + dx, t) - dist_out) / eps,
                (sd_scene(in_scene, test_pt + dy, t) - dist_out) / eps,
                (sd_scene(in_scene, test_pt + dz, t) - dist_out) / eps
            );
            hit = true;
            break;
        }
        motion += max(dist_out, 1e-4f);
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
        color.z = motion / 5.0f;
    } else if (render_cfg.debug_mode == 4) {
        color.xyz = normal;
    } else {
        if (hit) {
            const float n_shades = 3.0f;
            const float3 p = ray_origin + (ray * motion);
            const float3 light_1_pos = (float3) (2.0 * cos(0.1f * t), 2.0 * cos(0.1 * t), -5.0f);
            const float3 light_1_color = (float3) (0.9f, 0.2f, 0.1f);
            color.xyz += illuminate(in_scene, p, ray, normal, light_1_pos, light_1_color, t);

            const float3 light_2_pos = (float3) (2.0, 2.0, -3.0f);
            const float3 light_2_color = 0.0f * (float3) (0.9f, 0.2f, 0.1f);
            color.xyz += illuminate(in_scene, p, ray, normal, light_2_pos, light_2_color, t);
        } else {
            float rr = smallest_dist - 0.15;
            // color.y = 0.2 / (rr * rr)
            // color.y = 100.0 * exp(-35.0 * (rr * rr));
        }
    }

    const float3 power_adjusted_color = pow(clamp(color.xyz, 0.0f, 3.0f), 0.45f);

    float4 final_color = (float4) (power_adjusted_color, 1.0);
    int2 write_coord = px_coord;

    write_imagef(rendered_image, write_coord, 255.0f * final_color);
}