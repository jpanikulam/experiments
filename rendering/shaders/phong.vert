// TODO: Do this automatically
#version 460 core
layout (location = 0) in vec3 vertex_world;
layout (location = 1) in vec3 vertex_color;

out vec4 o_vertex_color;

uniform mat4x4 camera_from_world;
uniform mat4x4 perspective_from_camera;

void main()
{
    const vec4 vertex_camera = camera_from_world * vec4(vertex_world, 1.0);
    const vec4 vertex_perspective = perspective_from_camera * vertex_camera;

    gl_Position = vertex_perspective;
    o_vertex_color = vec4(vertex_color, 1.0);
}