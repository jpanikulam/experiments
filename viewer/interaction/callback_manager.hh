#pragma once

#include "geometry/ray.hh"
#include "geometry/shapes/line_segment.hh"

#include "viewer/interaction/keys.hh"

#include <functional>
#include <vector>

namespace viewer {

// --> GUI (Checkbox or button)
// --> Single key-press(With description)

class CallbackManager {
private:
  struct CallbackPair {
    ClickCallback callback;
    std::vector<LineSegment> segments;
  };
 public:
  using ClickCallback = std::function<bool(void )>;
  void register_callback(const ClickCallback& callback,
                         const std::vector<LineSegment>& line_segments) {
    callback_pairs.push_back({callback, line_segments});
  }



 private:
  std::vector<CallbackPair> callback_pairs;
  ;
};

}  // namespace viewer
