#pragma once

#include "viewer/gl_size.hh"

#include <GL/glew.h>
#include <cstddef>

namespace jcc {

class Texture {
 public:  //
  Texture();
  ~Texture();

  uint64_t get_id() const {
    return texture_id_;
  }

  void bind() const;

  void tex_image_2d(const GLenum target,
                    const GLint level,
                    const GLint internal_format,
                    const viewer::GlSize& size,
                    const GLint border,
                    const GLenum format,
                    const GLenum type,
                    GLvoid* const data = nullptr);

  void destroy();

  const viewer::GlSize& size() const {
    return size_;
  }

 private:
  viewer::GlSize size_;
  GLuint texture_id_ = 0;
};

}  // namespace jcc