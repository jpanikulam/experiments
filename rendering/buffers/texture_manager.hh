#pragma once

#include "rendering/buffers/texture.hh"

#include <map>

// TODO
#include <iostream>

namespace jcc {

//
// Contraption for asking questions about existing textures
// Features:
//  - Visualize any texture (including depth maps, silly)
//  - Make sure those damn textures don't leak by limiting the number of managed textures
//
class TextureManager {
 public:
  TextureManager();

  Texture& create_texture(const std::string& name);

  Texture& texture(const std::string& name) {
    return textures_.at(name);
  }

  void show_ui();

  void clear();

 private:
  std::map<std::string, Texture> textures_;

  // TODO always clear this
  std::map<std::string, double> texture_zoom_;
};

}  // namespace jcc