struct Scene {
    int n_objects;
    __global struct ComplexObject * objects;
};

float sd_object(__global struct ComplexObject obj, const float3 p) {
    float dist = 1000.0f;
    for (int k = 0; k < obj->n_u_sphere; ++k) {
        dist = union_sd(sd_sphere(obj->u_sphere[k], p), dist);
    }
    for (int k = 0; k < obj->n_u_plane; ++k) {
        dist = union_sd(sd_plane(obj->u_plane[k], p), dist);
    }
    for (int k = 0; k < obj->n_u_box; ++k) {
        dist = union_sd(sd_box(obj->u_box[k], p), dist);
    }

    for (int k = 0; k < obj->n_int_sphere; ++k) {
        dist = intersect_sd(sd_sphere(obj->int_sphere[k], p), dist);
    }
    for (int k = 0; k < obj->n_int_plane; ++k) {
        dist = intersect_sd(sd_plane(obj->int_plane[k], p), dist);
    }
    for (int k = 0; k < obj->n_int_box; ++k) {
        dist = intersect_sd(sd_box(obj->int_box[k], p), dist);
    }

    float dist_sub = 1000.0f;
    for (int k = 0; k < obj->n_sub_sphere; ++k) {
        dist_sub = union_sd(sd_sphere(obj->sub_sphere[k], p), dist_sub);
    }
    for (int k = 0; k < obj->n_sub_plane; ++k) {
        dist_sub = union_sd(sd_plane(obj->sub_plane[k], p), dist_sub);
    }
    for (int k = 0; k < obj->n_sub_box; ++k) {
        dist_sub = union_sd(sd_box(obj->sub_box[k], p), dist_sub);
    }

    dist = subtract_sd(dist, dist_sub);
    return dist;
}