#version 460 core
out vec4 FragColor;

in vec4 o_color;
in vec3 o_light_pos;
in vec3 o_normal_world;
in vec3 o_world_pos;
in vec3 o_vertex_camera;

uniform mat4x4 camera_from_world;
uniform mat4x4 perspective_from_camera;
uniform mat4x4 light_from_world;
uniform mat4x4 perspective_from_light;
uniform vec3 light_pos_world;
uniform sampler2D shadow_texture;

float compute_diffuse(const vec3 normal_world, const vec3 light_dir_world, const float light_power) {
    return light_power * clamp(dot(light_dir_world, normal_world), 0.0, 1.0);
}

float compute_specular(const vec3 normal_world, const vec3 light_dir_world, const vec3 view_dir_camera, const float light_power) {
    const vec3 reflection = reflect(light_dir_world, normal_world);
    const vec3 normal_camera = mat3(camera_from_world) * normal_world;
    const float visibility_mask = step(0.0, -dot(view_dir_camera, normal_camera));
    return light_power * pow(clamp(dot(reflection, view_dir_camera), 0.0, 1.0), 5.0);
}

void main() {
    const vec3 surface_color_diffuse = o_color.xyz;
    const vec3 normal_world = normalize(o_normal_world);

    const vec3 view_dir_camera = normalize(o_vertex_camera);

    const vec3 relative_light_pos = (light_pos_world - o_world_pos);
    const float light_dist = length(relative_light_pos);
    const vec3 light_dir_world = relative_light_pos / light_dist;

    const vec3 normal_camera = mat3(camera_from_world) * normal_world;
    const float visibility_mask = step(0.0, -dot(view_dir_camera, normal_camera));

    const float kd = 0.6;
    const float ks = 0.3;
    const float light_power = 1.0 / (light_dist * light_dist);
    const float diffuse = kd * compute_diffuse(normal_world, light_dir_world, light_power);
    const float specular = ks * compute_specular(normal_world, light_dir_world, view_dir_camera, light_power);
    const vec3 surface_color = visibility_mask * (diffuse * surface_color_diffuse) + (specular * surface_color_diffuse);

    const vec4 fragment_h_light_frame = perspective_from_light * (light_from_world * vec4(o_world_pos, 1.0));
    const vec3 fragment_light_frame = fragment_h_light_frame.xyz / fragment_h_light_frame.w;

    const vec3 fragment_pos_light_frame = (light_from_world * vec4(o_world_pos, 1.0)).xyz;

    const vec3 normal_light_frame = mat3(light_from_world) * normal_world;
    const float depth_light_frame = fragment_h_light_frame.z;

    const float light_backface_mask = step(0.0, dot(-fragment_pos_light_frame, normal_light_frame));
    // const float light_visibility_mask = step(0.0, fragment_light_frame.z) * light_backface_mask;
    const vec3 in_light_frame = step(vec3(-1.0, -1.0, -1.0), fragment_light_frame) * step(vec3(-1.0, -1.0, -1.0), -fragment_light_frame);
    const float light_visibility_mask = light_backface_mask * (in_light_frame.x * in_light_frame.y * in_light_frame.z);

    const vec2 tex_coord_light_frame = (0.5 * fragment_light_frame).xy + vec2(0.5, 0.5);
    const float shadow_norm_dist = texture(shadow_texture, tex_coord_light_frame.xy).x;

    // const float shadow_mask = step(shadow_norm_dist, 0.95 * depth_light_frame);
    // const float shadow_mask = abs(shadow_norm_dist - depth_light_frame);

    // const float shadow_mask = depth_light_frame;
    // const float shadow_mask = 10.0 * abs(depth_light_frame - shadow_norm_dist);
    const float shadow_mask = step(-0.1, -abs(depth_light_frame - shadow_norm_dist));

    // const vec3 color = surface_color * shadow_mask;
    vec3 color = surface_color;

    const vec4 light_camera = camera_from_world * vec4(light_pos_world, 1.0);
    const vec4 light_perspective = perspective_from_camera * light_camera;
    const vec2 light_projected = vec2(1920, 1080) * (((light_perspective.xy / light_perspective.w) * 0.5) + 0.5);

    color.z = 1.0 * step(gl_FragCoord.x - 50, light_projected.x) * step(-(gl_FragCoord.x + 50), -light_projected.x);
    color.z *= 1.0 * step(gl_FragCoord.y - 50, light_projected.y) * step(-(gl_FragCoord.y + 50), -light_projected.y);

    color.xyz *= max(0.1, light_visibility_mask * shadow_mask);
    // color.xyz = mix(color.xyz, )

    // vec3 color = vec3(0.0);
    // color.z = 1.0 * step(0.0, tex_coord_light_frame.x) * step(-1.0, -tex_coord_light_frame.x) *
    //           step(0.0, tex_coord_light_frame.y) * step(-1.0, -tex_coord_light_frame.y);

    // color.z = light_visibility_mask * step(-1.0, fragment_light_frame.x) * step(-1.0, -fragment_light_frame.x) *
    //           step(-1.0, fragment_light_frame.y) * step(-1.0, -fragment_light_frame.y);

    // color.xy = light_visibility_mask * tex_coord_light_frame;

    // color.xyz = vec3(light_visibility_mask * shadow_mask);

    // color.xyz *= texture(shadow_texture, gl_FragCoord.xy / vec2(1920, 1080)).xyz;
    // color.xyz += surface_color.xyz * 0.4;

    //
    // Gamma
    //
    const float gamma = 0.4545;
    const vec3 final_color = pow(clamp(color, 0.0, 1.0), vec3(gamma));
    // const vec3 final_color = color;

    FragColor = vec4(final_color.xyz, 1.0);
}