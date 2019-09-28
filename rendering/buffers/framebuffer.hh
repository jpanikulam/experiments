#pragma once

#include <GL/glew.h>

#include "rendering/buffers/texture.hh"

#include <vector>

namespace jcc {

// Capitalization consistent with opengl, despite that being nuts
class Framebuffer {
  //
 public:
  Framebuffer();
  ~Framebuffer();

  void bind() const;
  void destroy();

  void attach_color_texture(const Texture& texture);
  void attach_depth_texture(const Texture& texture);

  void draw_buffers() const;

  GLuint get_id() const {
    return framebuffer_id_;
  }

 private:
  std::vector<GLenum> draw_buffers_;
  GLuint framebuffer_id_ = 0;
  int n_color_attachments_ = 0;
};

}  // namespace jcc