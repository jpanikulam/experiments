#version 460 core
out vec4 FragColor;

in vec4 o_vertex_color;

void main()
{
    FragColor = o_vertex_color * gl_FragCoord.w;
    // FragColor = gl_FragCoord;
}