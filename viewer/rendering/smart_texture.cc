#include "viewer/rendering/smart_texture.hh"

#include "logging/assert.hh"

namespace viewer {

class SmartTexture::ManagedTexture {
 public:
  ManagedTexture() {
    glGenTextures(1, &texture_id_);
  }

  ~ManagedTexture() {
    glDeleteTextures(1u, &texture_id_);
  }

  void bind() const {
    glBindTexture(GL_TEXTURE_2D, texture_id_);
  }

 private:
  GLuint texture_id_ = 0;
};

// Takes ownership over a GL pointer
SmartTexture::SmartTexture(const jcc::Vec2& size) : size_(size) {
  managed_texture_ = std::make_shared<ManagedTexture>();
}

void SmartTexture::tex_image_2d(const GLenum target,
                                const GLint level,
                                const GLint internal_format,
                                const GLsizei width,
                                const GLsizei height,
                                const GLint border,
                                const GLenum format,
                                const GLenum type,
                                GLvoid* const data) const {
  JASSERT(managed_texture_.get(), "Managed texture must be instantiated");
  managed_texture_->bind();
  glTexImage2D(target, level, internal_format, width, height, border, format, type, data);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void SmartTexture::draw() const {
  managed_texture_->bind();

  glColor3d(1.0, 1.0, 1.0);
  glBegin(GL_QUADS);
  {
    glTexCoord2d(0.0, 0.0);
    glVertex2d(0.0, 0.0);

    glTexCoord2d(0.0, 1.0);
    glVertex2d(0.0, size_.y());

    glTexCoord2d(1.0, 1.0);
    glVertex2d(size_.x(), size_.y());

    glTexCoord2d(1.0, 0.0);
    glVertex2d(size_.x(), 0.0);
  }
  glEnd();
}
}  // namespace viewer