#pragma once

#include "eigen.hh"

#include <GL/glew.h>

#include <map>
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

  // Should we permit a way to build directly?
  // explicit SmartTexture(const ByteBuffer);
  // static make_texture(...);

 private:
  class ManagedTexture;
  // Really, this is the smart part
  std::shared_ptr<ManagedTexture> managed_texture_;
  jcc::Vec2 size_;
};

//        |      |____________ <- width
// height |      |
//        |      |
//        | b_x  |
//        | -^-- |
//        |      |            |<- b_y
//        |      |            |
//        |
//        |-------------------------> Advance
//
struct GlCharacter {
  SmartTexture texture;
  jcc::Vec2 dimensions;  // Size of glyph
  jcc::Vec2 bearing;     // Offset from baseline to left/top of glyph
  double advance;        // Offset to advance to next glyph
};

using CharacterLibrary = std::map<char, GlCharacter>;
CharacterLibrary create_text_library();

void write_string(const std::string& text,
                  const CharacterLibrary& lib,
                  const double size = 1.0);

}  // namespace viewer
