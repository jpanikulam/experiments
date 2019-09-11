#version 460 core
out vec4 FragColor;

in vec4 o_color;
in vec3 o_light_pos;
in vec3 o_normal;

uniform mat4x4 camera_from_world;

void main() {
    const vec3 surface_color_diffuse = o_color.xyz;

    // const vec3 relative_light_pos = o_light_pos - gl_FragCoord.xyz;
    const vec4 light_pos_world = vec4(10.0, 1.0, 1.0, 1.0);
    const vec3 relative_light_pos = (camera_from_world * light_pos_world).xyz;
    const float light_dist = length(relative_light_pos);

    const float fresnel = pow(clamp(1.0 + dot(o_normal, relative_light_pos / light_dist), 0.0, 1.0), 5.0);

    const float power_coeff = clamp(dot(o_normal, relative_light_pos / light_dist), 0.0, 1.0);
    const vec3 color = power_coeff * surface_color_diffuse;


    const vec3 brdf =
        // vec3() +
        vec3(0.5, 0.5, 0.5) +
        (0.7 * smoothstep(0.1, 0.15, fresnel))
    ;
    const vec3 interacted_color = brdf * surface_color_diffuse;

    //
    // Gamma
    //
    const float gamma = 0.4545;
    const vec3 final_color = pow(clamp(interacted_color, 0.0, 1.0), vec3(gamma));

    FragColor = vec4(final_color.xyz, 1.0);
    // FragColor = vec4(power_coeff * 0.1 + (0.001 * color), 1.0);
    // FragColor = gl_FragCoord;
}