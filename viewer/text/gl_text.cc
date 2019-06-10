#include "logging/assert.hh'"

//%deps(freetype)
#include <ft2build.h>
#include FT_FREETYPE_H

namespace {

// TODO
// - Finish this function
// - Write a thing that dumps out character textures (But does tex lookup get expensive)
// - Generate text and draw lines to it in a vertical box

// [1] https://learnopengl.com/In-Practice/Text-Rendering
void generate_character() {
  FT_Library ft;
  JASSERT(FT_Init_FreeType(&ft), "Failed initialize freetype");

  FT_Face face;
  JASSERT(FT_New_Face(ft, "fonts/arial.ttf", 0, &face), "Failed to load freetype font");

  // Test Load
  FT_Set_Pixel_Sizes(face, 0, 48);
  JASSERT(FT_Load_Char(face, 'X', FT_LOAD_RENDER), "Failed to load freetype glyph");
}

void boop() {
  for (GLubyte c = 0; c < 128; c++) {
    // Load character glyph
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // Disable byte-alignment restriction

    JASSERT(FT_Load_Char(face, c, FT_LOAD_RENDER),
            "Failed to load glyph for a character");

    // Generate texture
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_LUMINANCE,
                 face->glyph->bitmap.width,
                 face->glyph->bitmap.rows,
                 0,
                 GL_LUMINANCE,
                 GL_UNSIGNED_BYTE,
                 face->glyph->bitmap.buffer);
    // Set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Now store character for later use

    // Character character = {
    //     texture, glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
    //     glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
    //     face->glyph->advance.x};
    // Characters.insert(std::pair<GLchar, Character>(c, character));
  }

  FT_Done_Face(face);
  FT_Done_FreeType(ft);
}

}  // namespace

class GlText {
 public:
  GlText(const std::string& text) {
    constexpr int DEPTH = 3;
    buffer_ = std::make_unique(new unsigned char[cols * rows * DEPTH])
  }

 private:
  bool allocated_texture_ = false;
  int cols_ = -1;
  int rows_ = -1;

  // auto testData = std::unique_ptr<unsigned char[]>{ new unsigned char[16000] };
  std::unique_ptr<unsigned char[]> buffer_;

  double aspect_ratio = 1.0;
  double width_m_ = 1.0;
};
void GlText::update_gl() const {
  if (!allocated_texture_) {
    glGenTextures(1, &texture_id_);
    allocated_texture_ = true;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cols_, rows_, 0, GL_BGR, GL_UNSIGNED_BYTE,
               &data_);
}

void GlText::draw() const {
  if (to_update_) {
    update_gl();
    to_update_ = false;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glColor4d(1.0, 1.0, 1.0, alpha_);
  glBegin(GL_QUADS);

  const double aspect_ratio = image_.cols / static_cast<double>(image_.rows);

  const double height = aspect_ratio * width_m_;

  glTexCoord2d(1, 0);
  glVertex3d(height - (height * 0.5), 0 - (width_m_ * 0.5), 0);
  glTexCoord2d(1, 1);
  glVertex3d(height - (height * 0.5), width_m_ - (width_m_ * 0.5), 0);
  glTexCoord2d(0, 1);
  glVertex3d(0 - (height * 0.5), width_m_ - (width_m_ * 0.5), 0);
  glTexCoord2d(0, 0);
  glVertex3d(0 - (height * 0.5), 0 - (width_m_ * 0.5), 0);

  glEnd();

  glDisable(GL_TEXTURE_2D);
}