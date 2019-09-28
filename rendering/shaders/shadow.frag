#version 460 core

in vec3 o_vertex_world;
in vec3 o_normal_world;
in vec3 o_vertex_light;
in vec3 o_pos_light_frame;
in vec4 vertex_perspective;

layout(location = 0) out vec4 normal;

void main() {
    const vec3 normal_world = normalize(o_normal_world);
    normal = vec4(0.5 + (0.5 * o_normal_world), 1.0);

    // normal.x = step(-0.1, o_vertex_world.x) * step(-0.1, -o_vertex_world.x);;
    // normal.y = step(-0.1, o_vertex_world.y) * step(-0.1, -o_vertex_world.y);;
    // normal.z = step(-0.1, o_vertex_world.z) * step(-0.1, -o_vertex_world.z);;

    normal.xyz = vec3(abs(vertex_perspective.z));

    // frag_depth = gl_FragCoord.z;
    // gl_FragDepth = gl_FragCoord.z;

    // gl_FragDepth = -vertex_perspective.z / vertex_perspective.w;

    // gl_FragDepth = abs(cos(2.0 * o_vertex_world.x));
    // gl_FragDepth = 0.0;
}