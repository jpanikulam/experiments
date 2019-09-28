#version 460 core
layout (location = 0) in vec3 vertex_world;
layout (location = 1) in vec3 vertex_normal;

out vec3 o_vertex_world;
out vec3 o_vertex_light;
out vec3 o_normal_world;
out vec3 o_pos_light_frame;
out vec4 vertex_perspective;

uniform mat4x4 light_from_world;
uniform mat4x4 perspective_from_light;

void main() {
    const vec4 vertex_light = light_from_world * vec4(vertex_world, 1.0);
    vertex_perspective = perspective_from_light * vertex_light;

    o_pos_light_frame = vertex_light.xyz;

    gl_Position = vertex_perspective;
    o_vertex_world = vertex_world;
    o_normal_world = vertex_normal;
    o_vertex_light = (light_from_world * vec4(vertex_world, 1.0)).xyz;
}
