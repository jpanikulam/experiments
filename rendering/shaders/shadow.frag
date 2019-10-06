#version 460 core

in vec3 v_vertex_world;
in vec3 v_normal_world;
in vec3 v_vertex_light;
in vec3 v_pos_light_frame;
in vec3 v_vertex_color;

in vec3 v_normal_light;

in vec4 v_vertex_perspective;

// TODO: Check that we're getting this right
layout(location = 0) out vec4 normal;
layout(location = 1) out vec4 position;
layout(location = 2) out vec4 color;

void main() {
    const vec3 normal_world = normalize(v_normal_world);
    normal.xyz = normal_world;
    normal.w = v_vertex_perspective.z;

    position.xyz = v_vertex_world.xyz;
    position.w = 1.0;

    const float light_power = 1.0 / dot(v_vertex_light, v_vertex_light);
    const float lambert_coeff = clamp(dot(v_normal_light, -normalize(v_vertex_light.xyz)), 0.0, 1.0);

    const float kd = 0.6;
    const float return_power = kd * light_power * lambert_coeff;
    color.xyz = v_vertex_color * return_power;
    color.a = 1.0;
}