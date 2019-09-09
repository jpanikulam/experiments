#pragma once

#include "viewer/interaction/view3d.hh"
#include "viewer/gl_size.hh"


namespace viewer {

// OpenGL nominal matrix formats are usually described as follows:
//
// proj_from_view * view_from_world * world_from_model * pt_model_frame
// For us: all points are in the world frame, so world_from_model = I
//
// Which means `view_from_world` == `camera_from_anchor * anchor_from_world`
//
// OpenGL MultMatrix calls *right* multiply the view_from_world matrix
// by the supplied matrix
//
// Tracking state in the below computation:
// view_from_world = I
// view_from_world = view_from_world * camera_from_anchor;
//  -> (view_from_anchor)
// view_from_world = view_from_world * anchor_from_world;
//  -> (view_from_anchor * anchor_from_world)
//  -> (view_from_world)
//
void apply_view(const OrbitCamera &view);

void prepare_to_render();

void set_perspective(const GlSize &gl_size, bool ortho = false);

}  // namespace viewer