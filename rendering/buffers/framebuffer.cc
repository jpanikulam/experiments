#include "rendering/buffers/framebuffer.hh"

namespace jcc {

Framebuffer::Framebuffer() {
  glGenFramebuffers(1, &framebuffer_id_);
  bind();
}

Framebuffer::~Framebuffer() {
  if (framebuffer_id_ != 0u) {
    destroy();
  }
}

void Framebuffer::bind() const {
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_id_);
}

void Framebuffer::destroy() {
  glDeleteFramebuffers(1u, &framebuffer_id_);
  framebuffer_id_ = 0;
  draw_buffers_.clear();
  n_color_attachments_ = 0;
}

void Framebuffer::attach_color_texture(const Texture& texture) {
  const GLenum color_attachments[] = {
      GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2,
      GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5,
      GL_COLOR_ATTACHMENT6, GL_COLOR_ATTACHMENT7, GL_COLOR_ATTACHMENT8};

  glNamedFramebufferTexture(framebuffer_id_, color_attachments[n_color_attachments_],
                            texture.get_id(), 0);
  draw_buffers_.push_back(color_attachments[n_color_attachments_]);

  ++n_color_attachments_;
}

void Framebuffer::attach_depth_texture(const Texture& texture) {
  glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, texture.get_id(), 0);
}

void Framebuffer::draw_buffers() const {
  glNamedFramebufferDrawBuffers(framebuffer_id_, draw_buffers_.size(),
                                draw_buffers_.data());
}

}  // namespace jcc