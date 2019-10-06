// TODO: Do this automatically
#version 460 core
layout (location = 0) in vec3 vertex_world;
layout (location = 1) in vec3 vertex_color;
layout (location = 2) in vec3 vertex_normal;
layout (location = 3) in int material_id;

out vec4 v_color;
out vec3 v_normal_world;
out vec3 v_world_pos;
out vec3 v_vertex_camera;
flat out int f_material_id;

layout (location = 0) uniform vec3 light_pos_world;
layout (location = 1) uniform mat4x4 u_camera_from_world;
layout (location = 2) uniform mat4x4 u_perspective_from_camera;
layout (location = 3) uniform mat4x4 u_light_from_world;
layout (location = 4) uniform mat4x4 u_perspective_from_light;

void main() {
    const vec4 vertex_camera = u_camera_from_world * vec4(vertex_world, 1.0);
    const vec4 vertex_perspective = u_perspective_from_camera * vertex_camera;
    v_world_pos = vertex_world;

    gl_Position = vertex_perspective;
    v_vertex_camera = vertex_camera.xyz;

    v_color = vec4(vertex_color, 1.0);
    v_normal_world = vertex_normal;
    int f_material_id = material_id;
}
