#pragma once

#include "eigen.hh"

#include "viewer/rendering/smart_texture.hh"

#include <map>

namespace viewer {

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
