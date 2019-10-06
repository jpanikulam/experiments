#version 460 core
layout (location = 0) in vec3 vertex_world;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec3 vertex_color;

out vec3 v_vertex_world;
out vec3 v_vertex_color;

out vec3 v_vertex_light;
out vec3 v_normal_world;
out vec3 v_pos_light_frame;
out vec4 v_vertex_perspective;

out vec3 v_normal_light;

uniform mat4x4 u_light_from_world;
uniform mat4x4 u_perspective_from_light;

void main() {
    const vec4 vertex_light = u_light_from_world * vec4(vertex_world, 1.0);
    v_vertex_perspective = u_perspective_from_light * vertex_light;

    v_pos_light_frame = vertex_light.xyz;
    v_vertex_color = vertex_color;
    v_vertex_world = vertex_world;
    v_normal_world = vertex_normal;

    v_vertex_light = (u_light_from_world * vec4(vertex_world, 1.0)).xyz;
    v_normal_light = mat3(u_light_from_world) * vertex_normal;

    gl_Position = v_vertex_perspective;
}
