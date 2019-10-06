#version 460 core
out vec4 FragColor;

struct MaterialProperties {
    vec3 emissivity;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

const int N_MATERIALS = 16;
uniform MaterialProperties u_materials[N_MATERIALS];

flat in int f_material_id;

in vec4 v_color;
in vec3 v_light_pos;
in vec3 v_normal_world;
in vec3 v_world_pos;
in vec3 v_vertex_camera;

uniform mat4x4 u_camera_from_world;
uniform mat4x4 u_perspective_from_camera;
uniform mat4x4 u_light_from_world;
uniform mat4x4 u_perspective_from_light;
uniform vec3 u_light_pos_world;
uniform sampler2D u_shadow_normals_dist;
uniform sampler2D u_shadow_pos;
uniform sampler2D u_shadow_colors;

uniform bool u_dbg_shadows;
uniform bool u_dbg_misc;
uniform bool u_dbg_use_rsm;
uniform bool u_dbg_show_light_probes;

float vmax4(const vec4 v) {
    return max(max(v.x, v.y), max(v.z, v.w));
}

float bistep(const float bottom, const float top, const float value) {
    return step(bottom, value) * step(-top, -value);
}

float compute_diffuse(const vec3 normal_world, const vec3 dir_light_from_frag, const float light_power) {
    return light_power * clamp(dot(dir_light_from_frag, normal_world), 0.0, 1.0);
}

float compute_specular(const vec3 normal_world, const vec3 dir_light_from_frag, const vec3 view_dir_camera, const float light_power) {
    const vec3 reflection = reflect(dir_light_from_frag, normal_world);
    const vec3 normal_camera = mat3(u_camera_from_world) * normal_world;
    return light_power * pow(clamp(dot(reflection, view_dir_camera), 0.0, 1.0), 5.0);
}

float compute_shadow_mask(const vec3 normal_world) {
    const vec4 fragment_h_light_frame = u_perspective_from_light * (u_light_from_world * vec4(v_world_pos, 1.0));
    const vec3 fragment_light_frame = fragment_h_light_frame.xyz / fragment_h_light_frame.w;

    const vec3 fragment_pos_light_frame = (u_light_from_world * vec4(v_world_pos, 1.0)).xyz;

    const vec3 normal_light_frame = mat3(u_light_from_world) * normal_world;
    const float depth_light_frame = fragment_h_light_frame.z;

    const float light_backface_mask = step(0.0, dot(-fragment_pos_light_frame, normal_light_frame));
    const vec3 in_light_frame = step(vec3(-1.0, -1.0, -1.0), fragment_light_frame) * step(vec3(-1.0, -1.0, -1.0), -fragment_light_frame);
    const float light_visibility_mask = light_backface_mask * (in_light_frame.x * in_light_frame.y * in_light_frame.z);

    const vec2 tex_coord_light_frame = (0.5 * fragment_light_frame).xy + vec2(0.5, 0.5);
    const float shadow_norm_dist = texture(u_shadow_normals_dist, tex_coord_light_frame.xy).a;
    const float shadow_mask = step(-0.01, shadow_norm_dist - depth_light_frame);
    return light_visibility_mask * shadow_mask;
}

vec3 compute_rsm_color(const vec2 tex_coord, const vec3 normal_world) {
    const vec4 shadow_texel = texture(u_shadow_normals_dist, tex_coord.xy);
    const vec3 normal_world_shadow = shadow_texel.xyz;
    const vec3 pos_texel = texture(u_shadow_pos, tex_coord.xy).xyz;
    const vec4 color_texel = texture(u_shadow_colors, tex_coord.xy);

    const vec3 pos_light_probe = pos_texel + (normal_world_shadow * 0.01);
    // const vec3 pos_light_probe = pos_texel;

    const vec3 probe_from_frag = pos_light_probe - v_world_pos;
    const vec3 dir_probe_from_frag = normalize(probe_from_frag);

    const float light_power = 1.0 / (dot(probe_from_frag, probe_from_frag) + 0.4);
    const float direct_power = 0.01;

    const float reflect_power = clamp(dot(normal_world_shadow, -dir_probe_from_frag), 0.0, 1.0) *
                                clamp(dot(normal_world, dir_probe_from_frag), 0.0, 1.0);
    const float not_real_mask = step(-0.01, -color_texel.a);
    return not_real_mask * color_texel.rgb * reflect_power * light_power * light_power * direct_power;
}

const vec3 line_light(const vec3 position) {
    const vec3 line_light_color = vec3(0.1, 0.6, 0.1);
    const float light_power_w_per_m = 0.1;

    const vec3 p0 = vec3(0.0, 0.0, 1.0);
    const vec3 p1 = vec3(7.0, 0.0, 1.0);

    const float light_length = length(p1 - p0);
    const vec3 light_dir = (p1 - p0) / light_length;

    const vec3 error0 = p0 - position;
    const vec3 error1 = p1 - position;

    const vec3 dhat = normalize(error0 - (dot(error0, light_dir) * light_dir));

    // Error in the direction of the line start
    const float h = dot(dhat, error0);
    const float inv_h = 1.0 / h;

    const float s0 = dot(error0, light_dir);
    const float s1 = dot(error1, light_dir);

    const float integral = inv_h * (atan(s1 * inv_h) - atan(s0 * inv_h));
    return integral * light_power_w_per_m * line_light_color;
}

// https://www.wolframalpha.com/input/?i=integral+%28x+-+c%29+%2F+%28a+%2B+b*x*x%29+dx
// const vec3 light_light_2() {
//     float i1_1 = log(h3 + (h * s1));
//     float i1_2 = - 2.0 * sqrt_b * s0 * atan(sqrt_b * s1)
// }

void main() {
    //
    // Shared Precomputation
    //

    const vec3 surface_color_diffuse = v_color.xyz;
    const vec3 normal_world = normalize(v_normal_world);
    const vec3 view_dir_camera = normalize(v_vertex_camera);

    const dvec3 relative_light_pos = (u_light_pos_world - v_world_pos);
    // const float light_dist = float(length(relative_light_pos));
    const float light_dist = distance(u_light_pos_world, v_world_pos);
    // const vec3 dir_light_from_frag = relative_light_pos / light_dist;
    const vec3 dir_light_from_frag = vec3(normalize(relative_light_pos));

    const vec3 normal_camera = mat3(u_camera_from_world) * normal_world;
    const float visibility_mask = float (gl_FrontFacing);

    //
    // Mix diffuse and specular components
    //

    const float kd = 0.6;
    const float ks = 0.0;
    const float light_power = float(1.0 / dot(relative_light_pos, relative_light_pos));
    const float diffuse = kd * compute_diffuse(normal_world, dir_light_from_frag, light_power);
    const float specular = ks * compute_specular(normal_world, dir_light_from_frag, view_dir_camera, light_power);
    const vec3 surface_color = visibility_mask * (diffuse * surface_color_diffuse) + (specular * surface_color_diffuse);


    //
    // Line light
    //


    //
    // Visualize the light emitter itself
    //

    const vec4 light_camera = u_camera_from_world * vec4(u_light_pos_world, 1.0);
    const vec4 light_perspective = u_perspective_from_camera * light_camera;

    // *wink wink*
    const vec2 light_projected = vec2(1920, 1056) * (((light_perspective.xy / light_perspective.w) * 0.5) + 0.5);
    const float light_pixels = bistep(gl_FragCoord.x - 50, gl_FragCoord.x + 50, light_projected.x) *
                               bistep(gl_FragCoord.y - 50, gl_FragCoord.y + 50, light_projected.y) *
                               step(0.0, light_perspective.z);

    vec3 color = surface_color;

    color.z = max(color.z, light_pixels);

    //
    // Apply Shadows
    //

    if (u_dbg_shadows) {
        const float shadow_mask = compute_shadow_mask(normal_world);
        // color.xyz *= max(0.01, shadow_mask);
        color.xyz *= shadow_mask;
    }

    //
    // Line light without diffuse effects
    //

    color += (surface_color) * line_light(v_world_pos);

    if (false) {
        const vec4 fragment_h_light_frame = u_perspective_from_light * (u_light_from_world * vec4(v_world_pos, 1.0));
        const vec3 fragment_light_frame = fragment_h_light_frame.xyz / fragment_h_light_frame.w;

        const vec3 fragment_pos_light_frame = (u_light_from_world * vec4(v_world_pos, 1.0)).xyz;

        const vec3 normal_light_frame = mat3(u_light_from_world) * normal_world;
        const float depth_light_frame = fragment_h_light_frame.z;

        const float light_backface_mask = step(0.0, dot(-fragment_pos_light_frame, normal_light_frame));
        const vec3 in_light_frame = step(vec3(-1.0, -1.0, -1.0), fragment_light_frame) * step(vec3(-1.0, -1.0, -1.0), -fragment_light_frame);
        const float light_visibility_mask = light_backface_mask * (in_light_frame.x * in_light_frame.y * in_light_frame.z);

        const vec2 tex_coord_light_frame = (0.5 * fragment_light_frame).xy + vec2(0.5, 0.5);

        const vec3 normal_world_shadow = texture(u_shadow_normals_dist, tex_coord_light_frame.xy).xyz;
        const float shadow_norm_dist = texture(u_shadow_normals_dist, tex_coord_light_frame.xy).a;
        const float shadow_mask = step(-0.01, shadow_norm_dist - depth_light_frame);

        const vec4 neighbors = textureGather(u_shadow_normals_dist, tex_coord_light_frame.xy, 3);
        color.z = abs(vmax4(neighbors - vec4(shadow_norm_dist)));

        const vec3 pos_shadow = texture(u_shadow_pos, tex_coord_light_frame).xyz;
        color.x = length(pos_shadow - v_world_pos);
    }
    // color.y += u_materials[int(color.y)].emissivity.x;

    if (u_dbg_use_rsm) {
        // const vec4 fragment_h_light_frame = u_perspective_from_light * (u_light_from_world * vec4(v_world_pos, 1.0));
        // const vec3 fragment_light_frame = fragment_h_light_frame.xyz / fragment_h_light_frame.w;
        // const vec2 tex_coord_light_frame = (0.5 * fragment_light_frame).xy + vec2(0.5, 0.5);

        const float max_v = 1.0;
        const float step_size = 0.075;
        for (float i = 0.0; i <= max_v; i += step_size) {
            for (float j = 0.0; j <= max_v; j += step_size) {
                const vec2 tex_coord = vec2(i, j);
                const vec3 appl_color = compute_rsm_color(tex_coord, normal_world);
                color += appl_color;
                if (u_dbg_show_light_probes) {
                    const vec2 tex_coord = vec2(i, j);
                    const vec3 pos_texel = texture(u_shadow_pos, tex_coord.xy).xyz;
                    color.x += step(-0.005, -length(pos_texel - v_world_pos));
                }
            }
        }
    }

    //
    // Gamma
    //
    const float inv_gamma = 0.4545;


    vec3 final_color = pow(clamp(color, 0.0, 1.0), vec3(inv_gamma));

    // CONSIDER: (The error is pretty small!)
    // final_color = sqrt(clamp(color, 0.0, 1.0));

    FragColor = vec4(final_color.xyz, 1.0);
}