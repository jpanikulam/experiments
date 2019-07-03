#include "viewer/text/gl_text.hh"

#include "logging/assert.hh"

//%deps(freetype)
#include <ft2build.h>
#include FT_FREETYPE_H

namespace viewer {

namespace {

CharacterLibrary create_font_textures(const FT_Face& face) {
  CharacterLibrary textures;

  for (GLubyte character_id = 0; character_id < 128; character_id++) {
    if (FT_Load_Char(face, character_id, FT_LOAD_RENDER)) {
      continue;
    }

    //
    // Populate both a luminance and an alpha channel
    //

    unsigned char image[face->glyph->bitmap.rows][face->glyph->bitmap.width][2];
    const auto& buffer = face->glyph->bitmap.buffer;
    const int row_ct = face->glyph->bitmap.rows;
    const int cols_per_row = face->glyph->bitmap.width;

    // TODO(jpanikulam): Figure out how to memcpy it when less hungry
    for (int row = 0; row < row_ct; ++row) {
      for (int col = 0; col < cols_per_row; ++col) {
        image[row][col][0] = buffer[cols_per_row * row + col];
        image[row][col][1] = buffer[cols_per_row * row + col];
      }
    }

    //
    // Generate the actual texture
    //

    // Disable byte-alignment restriction
    // TODO(jpanikulam): Understand this more clearly
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    constexpr double SCALING_FACTOR = 0.0005;
    const jcc::Vec2 dimensions = SCALING_FACTOR * jcc::Vec2(cols_per_row, row_ct);
    SmartTexture smart_tex(dimensions);

    // TODO(jpanikulam): Don't let the driver pick a format
    smart_tex.tex_image_2d(GL_TEXTURE_2D,
                           0,
                           GL_LUMINANCE_ALPHA,
                           face->glyph->bitmap.width,
                           face->glyph->bitmap.rows,
                           0,
                           GL_LUMINANCE_ALPHA,
                           GL_UNSIGNED_BYTE,
                           image);

    //
    // Store the character texture
    //

    constexpr long int ADVANCE_PER_PX = 64;
    JASSERT_GT(
        face->glyph->advance.x, 0, "Jake assumed that 'advance' is strictly positive");
    const double advance = static_cast<double>(face->glyph->advance.x) / ADVANCE_PER_PX;
    // static_cast<std::size_t>(face->glyph->advance.x / ADVANCE_PER_PX);
    textures[character_id] = GlCharacter{
        .texture = smart_tex,      //
        .dimensions = dimensions,  //
        .bearing = SCALING_FACTOR *
                   jcc::Vec2(face->glyph->bitmap_left, face->glyph->bitmap_top),  //
        .advance = SCALING_FACTOR * advance                                       //
    };
  }

  JASSERT_FALSE(
      textures.empty(),
      "Failed to load any character glyphs. Probably, we failed to find a font.");

  return textures;
}  // namespace

}  // namespace

// [1] https://learnopengl.com/In-Practice/Text-Rendering
CharacterLibrary create_text_library() {
  FT_Library ft;
  JASSERT_EQ(FT_Init_FreeType(&ft), 0, "Failed initialize freetype");

  FT_Face face;
  // const std::string font = "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf";
  const std::string font = "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf";
  JASSERT_EQ(FT_New_Face(ft, font.c_str(), 0, &face), 0, "Failed to load freetype font");

  // Test Load
  JASSERT_EQ(FT_Set_Pixel_Sizes(face, 0, 48), 0, "Failed to set pixel sizes");
  constexpr int DPI = 1500;
  constexpr int FLG_MATCH_OTHER = 0;
  JASSERT_EQ(FT_Set_Char_Size(face, 10 * 64, FLG_MATCH_OTHER, DPI, FLG_MATCH_OTHER),
             0,
             "Failed to set char size");

  JASSERT_EQ(FT_Load_Char(face, 'X', FT_LOAD_RENDER), 0, "Failed to load freetype glyph");

  const auto characters = create_font_textures(face);

  FT_Done_Face(face);
  FT_Done_FreeType(ft);
  return characters;
}

void write_string(const std::string& text,
                  const CharacterLibrary& lib,
                  const double size) {
  glPushAttrib(GL_ENABLE_BIT);
  glEnable(GL_TEXTURE_2D);

  glPushMatrix();
  double row_length = 0;
  double row_height = 0;
  for (std::size_t i = 0u; i < text.size(); ++i) {
    const char c = text[i];

    if (c == '\n') {
      if (i + 1 == text.size()) {
        break;
      }

      glTranslated(-row_length, row_height + (size * 0.0005 * 35.0), 0.0);
      row_length = 0;
      row_height = 0;
      continue;
    }

    const auto character = lib.at(c);
    glTranslated(character.bearing.x(), -(character.dimensions.y() - character.bearing.y()), 0.0);
    character.texture.draw();
    glTranslated(character.advance - character.bearing.x(), (character.dimensions.y() - character.bearing.y()), 0.0);
    row_length += character.advance;
    row_height = std::max(row_height, character.dimensions.y());
  }
  glPopAttrib();
  glPopMatrix();
}

}  // namespace viewer
