#pragma once

#include "eigen.hh"

#include "rendering/buffers/texture.hh"
#include "rendering/buffers/vertex_array_object.hh"
#include "rendering/shaders/attribute_description.hh"

#include <cstddef>
#include <map>
#include <string>

namespace jcc {

// TODO
//
// template <int N>
// class VertexArray {
// };

class Shader {
 public:
  Shader() = default;

  explicit Shader(const int program_id);

  int get_id() const {
    return program_id_;
  }

  // TODO: Shader parameter update block
  // TODO: Destroy

  void use() const;

  //
  void set(const std::string& name, const Texture& texture) const;

  void set_float(const std::string& name, const float arg) const;
  void set_uint(const std::string& name, const std::size_t arg) const;
  void set_bool(const std::string& name, const bool arg) const;
  //
  // These functions are auto-generated
  //

  void set(const std::string& name, const MatNf<2, 3>& arg) const;
  void set(const std::string& name, const MatNf<2, 4>& arg) const;
  void set(const std::string& name, const MatNf<3, 2>& arg) const;
  void set(const std::string& name, const MatNf<3, 4>& arg) const;
  void set(const std::string& name, const MatNf<4, 2>& arg) const;
  void set(const std::string& name, const MatNf<3, 3>& arg) const;
  void set(const std::string& name, const VecNf<2>& arg) const;
  void set(const std::string& name, const VecNf<3>& arg) const;
  void set(const std::string& name, const VecNf<4>& arg) const;
  void set(const std::string& name, const MatNf<4, 4>& arg) const;
  void set(const std::string& name, const MatNf<4, 3>& arg) const;
  void set(const std::string& name, const MatNf<2, 2>& arg) const;

  VertexArrayObject generate_vao() const {
    return VertexArrayObject(attribute_from_name_);
  }

 private:
  int program_id_ = -1;

  std::map<std::string, UniformDescription> uniform_from_name_;
  std::map<std::string, AttributeDescription> attribute_from_name_;
  std::map<std::string, int> tex_unit_from_texture_;

  bool debug_mode_ = true;
};

}  // namespace jcc