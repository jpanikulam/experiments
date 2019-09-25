// TODO: Do this automatically
#version 460 core
layout (location = 0) in vec3 vertex_world;
layout (location = 1) in vec3 vertex_color;
layout (location = 2) in vec3 vertex_normal;

out vec4 o_color;
out vec3 o_normal_world;
out vec3 o_world_pos;

uniform mat4x4 camera_from_world;
uniform mat4x4 perspective_from_camera;

void main() {
    const vec4 vertex_camera = camera_from_world * vec4(vertex_world, 1.0);
    const vec4 vertex_perspective = perspective_from_camera * vertex_camera;
    o_world_pos = vertex_world;

    gl_Position = vertex_perspective;
    o_color = vec4(vertex_color, 1.0);
    // o_normal_world = transpose(mat3(camera_from_world)) * vertex_normal;
    // o_normal_world = mat3(camera_from_world) * vertex_normal;
    o_normal_world = vertex_normal;
}
