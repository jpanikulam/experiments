#include "rendering/buffers/texture.hh"

#include "logging/assert.hh"

namespace jcc {

Texture::Texture() {
  glGenTextures(1, &texture_id_);
  bind();

}

Texture::~Texture() {
  if (texture_id_ != 0) {
    destroy();
  }
}

void Texture::bind() const {
  glBindTexture(GL_TEXTURE_2D, texture_id_);
}

void Texture::tex_image_2d(const GLenum target,
                           const GLint level,
                           const GLint internal_format,
                           const viewer::GlSize& size,
                           const GLint border,
                           const GLenum format,
                           const GLenum type,
                           GLvoid* const data) {
  size_ = size;
  JASSERT_NE(texture_id_, 0u, "Texture ID cannot be zero");
  glTexImage2D(target, level, internal_format, size.width, size.height, border, format, type, data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void Texture::destroy() {
  glDeleteTextures(1u, &texture_id_);
  texture_id_ = 0;
}

}  // namespace jcc