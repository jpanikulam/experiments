#include "viewer/interaction/callback_manager.hh"

#include "geometry/intersection/ray_closest_approach.hh"

namespace viewer {

void CallbackManager::register_click_callback(
    const ClickCallback& callback,
    const std::vector<geometry::shapes::LineSegment>& line_segments) {
  segment_callback_pairs_.push_back({callback, line_segments});
}

void CallbackManager::register_click_callback(const ClickCallback& callback,
                                              const geometry::Plane& plane) {
  plane_callback_pairs_.push_back({callback, {plane}});
}

// TODO: Need to do some kind of screens-space solver
// TODO: Decide whether to expose mouse position to the callback
void CallbackManager::handle_callbacks(const geometry::Ray& ray,
                                       const ViewportPoint& mouse_pt) {
  handle_segment_callbacks(ray, mouse_pt);
  handle_plane_callbacks(ray, mouse_pt);
}

void CallbackManager::handle_segment_callbacks(const geometry::Ray& ray,
                                               const ViewportPoint& mouse_pt) {
  for (const auto& callback_pair : segment_callback_pairs_) {
    const int elements_ct = static_cast<int>(callback_pair.elements.size());

    for (int element_index = 0; element_index < elements_ct; ++element_index) {
      const auto& segment = callback_pair.elements[element_index];
      const auto result = line_ray_closest_approach(ray, segment);
      if (result) {
        callback_pair.callback(element_index,
                               result->squared_distance,
                               ray(result->along_ray),
                               result->on_line,
                               mouse_pt);
      }
    }
  }
}

void CallbackManager::handle_plane_callbacks(const geometry::Ray& ray,
                                             const ViewportPoint& mouse_pt) {
  for (const auto& callback_pair : plane_callback_pairs_) {
    const int elements_ct = static_cast<int>(callback_pair.elements.size());

    for (int element_index = 0; element_index < elements_ct; ++element_index) {
      const auto& segment = callback_pair.elements[element_index];

      jcc::Vec3 pt;
      const bool intersected = segment.intersect(ray, out(pt));

      if (intersected) {
        callback_pair.callback(element_index, 0.0, pt, pt, mouse_pt);
      }
    }
  }
}

void CallbackManager::clear_callbacks() {
  segment_callback_pairs_.clear();
  plane_callback_pairs_.clear();
}

}  // namespace viewer
