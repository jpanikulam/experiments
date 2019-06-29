#pragma once

#include "geometry/shapes/line_segment.hh"
#include "geometry/plane.hh"
#include "geometry/shapes/ray.hh"

#include "viewer/gl_types.hh"
#include "viewer/interaction/keys.hh"

#include <functional>
#include <vector>

namespace viewer {

// --> GUI (Checkbox or button)
// --> Single key-press(With description)
// --> "ClickableObject" manager

class CallbackManager {
 public:
  // int           : Index
  // double        : SQUARED distance
  // jcc::Vec3     : Point on the ray that was nearest
  // jcc::Vec3     : Point on the line segment that was clicked
  // ViewportPoint : Point on the screen where the mouse is
  using ClickCallback = std::function<void(
      int, double, const jcc::Vec3&, const jcc::Vec3&, const ViewportPoint&)>;
  void register_click_callback(
      const ClickCallback& callback,
      const std::vector<geometry::shapes::LineSegment>& line_segments);

  void register_click_callback(const ClickCallback& callback,
                               const geometry::Plane& plane);

  // TODO: Need to do some kind of screens-space solver
  // TODO: Decide whether to expose mouse position to the callback
  void handle_callbacks(const geometry::Ray& ray, const ViewportPoint& mouse_pt);

  void handle_segment_callbacks(const geometry::Ray& ray, const ViewportPoint& mouse_pt);

  void handle_plane_callbacks(const geometry::Ray& ray, const ViewportPoint& mouse_pt);

  void clear_callbacks();

 private:
  template <typename Shape>
  struct CallbackPair {
    ClickCallback callback;
    std::vector<Shape> elements;
  };
  std::vector<CallbackPair<geometry::shapes::LineSegment>> segment_callback_pairs_;
  std::vector<CallbackPair<geometry::Plane>> plane_callback_pairs_;
};

}  // namespace viewer
