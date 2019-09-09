#pragma once

#include <string>

#include "rendering/shaders/shader.hh"

namespace jcc {

Shader load_shaders(const std::string& vertex_path, const std::string& frag_path);

}  // namespace jcc
