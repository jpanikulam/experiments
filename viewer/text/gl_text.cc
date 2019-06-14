#include "viewer/text/gl_text.hh"

#include "logging/assert.hh"

#include <memory>

//%deps(freetype)
#include <ft2build.h>
#include FT_FREETYPE_H

namespace viewer {

class SmartTexture::ManagedTexture {
 public:
  // ManagedTexture() = default;
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

namespace {

CharacterLibrary create_font_textures(const FT_Face& face) {
  CharacterLibrary textures;

  for (GLubyte character_id = 0; character_id < 128; character_id++) {
    // Load character glyph
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // Disable byte-alignment restriction

    if (FT_Load_Char(face, character_id, FT_LOAD_RENDER)) {
      continue;
    }

    // Generate texture
    // GLuint tex_id;
    // glGenTextures(1, &tex_id);
    const jcc::Vec2 dimensions =
        jcc::Vec2(face->glyph->bitmap.width, face->glyph->bitmap.rows);
    SmartTexture smart_tex(dimensions);

    // GL_LUMINANCE_ALPHA
    smart_tex.tex_image_2d(GL_TEXTURE_2D,
                           0,
                           GL_LUMINANCE,
                           face->glyph->bitmap.width,
                           face->glyph->bitmap.rows,
                           0,
                           GL_LUMINANCE,
                           GL_UNSIGNED_BYTE,
                           face->glyph->bitmap.buffer);

    // Store character for later use

    constexpr long int ADVANCE_PER_PX = 64;
    JASSERT_GT(
        face->glyph->advance.x, 0, "Jake assumed that 'advance' is strictly positive");
    const auto advance =
        static_cast<std::size_t>(face->glyph->advance.x / ADVANCE_PER_PX);
    textures[character_id] = GlCharacter{
        .texture = smart_tex,                                                     //
        .dimensions = dimensions,                                                 //
        .bearing = jcc::Vec2(face->glyph->bitmap_left, face->glyph->bitmap_top),  //
        .advance = advance                                                        //
    };
  }

  JASSERT_FALSE(
      textures.empty(),
      "Failed to load any character glyphs. Probably, we failed to find a font.");

  return textures;
}

}  // namespace

// [1] https://learnopengl.com/In-Practice/Text-Rendering
CharacterLibrary create_text_library() {
  FT_Library ft;
  JASSERT_EQ(FT_Init_FreeType(&ft), 0, "Failed initialize freetype");

  FT_Face face;
  const std::string libre_mono =
      "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf";
  JASSERT_EQ(
      FT_New_Face(ft, libre_mono.c_str(), 0, &face), 0, "Failed to load freetype font");

  // Test Load
  FT_Set_Pixel_Sizes(face, 0, 48);
  JASSERT_EQ(FT_Load_Char(face, 'X', FT_LOAD_RENDER), 0, "Failed to load freetype glyph");

  const auto characters = create_font_textures(face);

  FT_Done_Face(face);
  FT_Done_FreeType(ft);
  return characters;
}

void write_string(const std::string& text, const CharacterLibrary& lib) {
  glPushAttrib(GL_ENABLE_BIT);
  glEnable(GL_TEXTURE_2D);

  for (std::size_t i = 0u; i < text.size(); ++i) {
    const char c = text[i];
    const auto character = lib.at(c);

    // Debug stuff
    /*
    std::cout << "Drawing: " << c << std::endl;
    std::cout << "\t"
              << "Bearing: " << character.bearing.transpose() << std::endl;
    std::cout << "\t"
              << "Dims   : " << character.dimensions.transpose() << std::endl;
    std::cout << "\t"
              << "Advance: " << character.advance << std::endl;
    std::cout << "\n---" << std::endl;
    */
    glTranslated(character.bearing.x(), -character.bearing.y(), 0.0);
    character.texture.draw();
    glTranslated(character.advance - character.bearing.x(), character.bearing.y(), 0.0);
  }
  glPopAttrib();
}

}  // namespace viewer
