#pragma once

#include "viewer/gl_types.hh"
#include "viewer/primitives/primitive.hh"
#include "viewer/text/gl_text.hh"

#include <mutex>
#include <vector>

namespace viewer {

struct PointerTarget {
  std::string text;
  jcc::Vec3 world_pos;
  ViewportPoint location;
};

class Ui2d final : public Primitive {
 public:
  Ui2d() = default;
  void draw() const override;

  void add_pointer_target(const PointerTarget& pointer_target);

  void clear();

  void flush();
  void flip();

 private:
  struct Buffer {
    std::vector<PointerTarget> pointer_targets;

    void clear() {
      pointer_targets.clear();
    }
  };

  mutable CharacterLibrary char_lib_;

  Buffer back_buffer_;
  Buffer front_buffer_;

  mutable std::mutex draw_mutex_;
};

}  // namespace viewer
