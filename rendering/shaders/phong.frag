#version 460 core
out vec4 FragColor;

in vec4 o_color;
in vec3 o_light_pos;
in vec3 o_normal_world;
in vec3 o_world_pos;

uniform mat4x4 camera_from_world;

void main() {
    const vec3 surface_color_diffuse = o_color.xyz;
    const vec3 normal_world = normalize(o_normal_world);

    const vec3 light_pos_world = vec3(10.0, 1.0, 1.0);

    const vec3 pt_world = (camera_from_world * vec4(o_world_pos, 1.0)).xyz;
    const vec3 view_dir = normalize(pt_world);

    const vec3 relative_light_pos = (light_pos_world - o_world_pos);
    const float light_dist = length(relative_light_pos);
    const vec3 light_dir_world = relative_light_pos / light_dist;

    const vec3 reflection = reflect(light_dir_world, normal_world);

    const float kd = 0.6;
    const float ks = 0.3;

    const float diffuse = kd * clamp(dot(light_dir_world, normal_world), 0.0, 1.0);
    const float specular = ks * pow(clamp(dot(reflection, view_dir), 0.0, 1.0), 5.0);
    const vec3 color = (diffuse * surface_color_diffuse) + (specular * surface_color_diffuse);


    //
    // Gamma
    //
    const float gamma = 0.4545;
    const vec3 final_color = pow(clamp(color, 0.0, 1.0), vec3(gamma));

    FragColor = vec4(final_color.xyz, 1.0);
    // FragColor = vec4(o_normal_world, 1.0);
}