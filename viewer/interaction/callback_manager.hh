#pragma once

#include "geometry/intersection/ray_closest_approach.hh"
#include "geometry/shapes/line_segment.hh"
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
      const std::vector<geometry::shapes::LineSegment>& line_segments) {
    callback_pairs_.push_back({callback, line_segments});
  }

  // TODO: Need to do some kind of screens-space solver
  // TODO: Decide whether to expose mouse position to the callback
  void handle_callbacks(const geometry::Ray& ray, const ViewportPoint& mouse_pt) {
    for (const auto& callback_pair : callback_pairs_) {
      const int segments_ct = static_cast<int>(callback_pair.segments.size());

      for (int segment_index = 0; segment_index < segments_ct; ++segment_index) {
        const auto& segment = callback_pair.segments[segment_index];
        const auto result = line_ray_closest_approach(ray, segment);
        if (result) {
          callback_pair.callback(segment_index,
                                 result->squared_distance,
                                 ray(result->along_ray),
                                 result->on_line,
                                 mouse_pt);
        }
      }
    }
  }

  void clear_callbacks() {
    callback_pairs_.clear();
  }

 private:
  struct CallbackPair {
    ClickCallback callback;
    std::vector<geometry::shapes::LineSegment> segments;
  };
  std::vector<CallbackPair> callback_pairs_;
};

}  // namespace viewer
