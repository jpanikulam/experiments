#pragma once

#include "eigen.hh"

#include <GL/glew.h>
#include <memory>

namespace viewer {

// TODO: Factor this into a different TU, hide the gl/glew dependency
class SmartTexture {
 public:
  SmartTexture() = default;
  explicit SmartTexture(const jcc::Vec2& size);

  void tex_image_2d(const GLenum target,
                    const GLint level,
                    const GLint internal_format,
                    const GLsizei width,
                    const GLsizei height,
                    const GLint border,
                    const GLenum format,
                    const GLenum type,
                    GLvoid* const data) const;

  // The texture must be configured when this is called, but we expect it to
  // be called frequently enough that we're wary about assertion overhead
  void draw() const;

  bool ready() const {
    return static_cast<bool>(managed_texture_);
  }

  // Should we permit a way to build directly?
  // explicit SmartTexture(const ByteBuffer);
  // static make_texture(...);

 private:
  class ManagedTexture;
  // Really, this is the smart part
  std::shared_ptr<ManagedTexture> managed_texture_;
  jcc::Vec2 size_ = jcc::Vec2(1.0, 1.0);
};

}  // namespace viewer