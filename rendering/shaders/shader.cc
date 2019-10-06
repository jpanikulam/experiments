#include <GL/glew.h>

#include "logging/assert.hh"
#include "rendering/gl_interface/string_from_gl_type.hh"
#include "rendering/shaders/shader.hh"

namespace jcc {

// class UniformBuffer {
//   UniformBuffer() {
//     //
//   }
//   //
//   //
//   //
// };

Shader::Shader(const int program_id) : program_id_(program_id) {
  GLint attrib_count = -1;
  { glGetProgramiv(program_id, GL_ACTIVE_ATTRIBUTES, &attrib_count); }

  constexpr GLsizei buf_size = 128;
  for (GLint k = 0; k < attrib_count; ++k) {
    GLint size;
    GLenum type;
    GLchar name[buf_size];
    GLsizei name_length;
    glGetActiveAttrib(program_id, k, buf_size, &name_length, &size, &type, name);

    const std::string str_name(name);
    JASSERT_EQ(str_name.size(), static_cast<std::size_t>(name_length),
               "Name was not equal to gl specified length");
    const GLint location = glGetAttribLocation(program_id_, str_name.c_str());

    jcc::Info() << "Found attribute: " << str_name << ":" << std::endl;
    jcc::Info() << "\tType: " << string_from_gl_type(type) << std::endl;
    jcc::Info() << "\tSize: " << size << std::endl;
    jcc::Info() << "\tLocation: " << location << std::endl;

    AttributeDescription desc;
    desc.location = location;
    desc.size = size;
    desc.type = type;

    attribute_from_name_[str_name] = desc;
  }

  GLint uniform_count = -1;
  { glGetProgramiv(program_id, GL_ACTIVE_UNIFORMS, &uniform_count); }

  for (GLint k = 0; k < uniform_count; ++k) {
    GLint size;
    GLenum type;
    GLchar name[buf_size];
    GLsizei name_length;

    glGetActiveUniform(program_id, k, buf_size, &name_length, &size, &type, name);

    const std::string str_name(name);
    JASSERT_EQ(str_name.size(), static_cast<std::size_t>(name_length),
               "Name was not equal to gl specified length");

    const GLint location = glGetUniformLocation(program_id, str_name.c_str());

    jcc::Info() << "Found Uniform: " << str_name << ":" << std::endl;
    jcc::Info() << "\tType:" << string_from_gl_type(type) << std::endl;
    jcc::Info() << "\tSize: " << size << std::endl;
    jcc::Info() << "\tLocation: " << location << std::endl;

    if (type == GL_SAMPLER_2D) {
      const int n_textures = tex_unit_from_texture_.size();
      tex_unit_from_texture_[str_name] = n_textures;
      jcc::Info() << "\tAssigning to Texture Unit " << n_textures << std::endl;
    }

    UniformDescription desc;
    desc.location = glGetUniformLocation(program_id, str_name.c_str());
    desc.size = size;
    desc.type = type;
    uniform_from_name_[str_name] = desc;
  }
}

void Shader::use() const {
  glUseProgram(program_id_);
}

void Shader::set(const std::string& name, const Texture& texture) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const GLenum active_textures[] = {
      GL_TEXTURE0, GL_TEXTURE1, GL_TEXTURE2, GL_TEXTURE3, GL_TEXTURE4,
      GL_TEXTURE5, GL_TEXTURE6, GL_TEXTURE7, GL_TEXTURE8
      //
  };

  const int texture_unit = tex_unit_from_texture_.at(name);
  glActiveTexture(active_textures[texture_unit]);
  texture.bind();

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_SAMPLER_2D), "Mismatched argument type");
  glUniform1i(desc.location, texture_unit);
}

void Shader::set_float(const std::string& name, const float arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT), "Mismatched argument type");
  glUniform1f(desc.location, arg);
}

void Shader::set_uint(const std::string& name, const std::size_t arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  // TODO: Make this typesafe
  glUniform1i(desc.location, arg);
}

void Shader::set_bool(const std::string& name, const bool arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_BOOL), "Mismatched argument type");
  glUniform1i(desc.location, arg);
}

//
//
//

void Shader::set(const std::string& name, const MatNf<2, 3>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2x3), "Mismatched argument type");
  glUniformMatrix2x3fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<2, 4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2x4), "Mismatched argument type");
  glUniformMatrix2x4fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<3, 2>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3x2), "Mismatched argument type");
  glUniformMatrix3x2fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<3, 4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3x4), "Mismatched argument type");
  glUniformMatrix3x4fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<4, 2>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4x2), "Mismatched argument type");
  glUniformMatrix4x2fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<3, 3>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3), "Mismatched argument type");
  glUniformMatrix3fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const VecNf<2>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC2), "Mismatched argument type");
  glUniform2fv(desc.location, 1, arg.data());
}

void Shader::set(const std::string& name, const VecNf<3>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC3), "Mismatched argument type");

  glProgramUniform3fv(program_id_, desc.location, 1, arg.data());
  GLenum err;
  err = glGetError();
  JASSERT_EQ(err, GL_NO_ERROR, "There was  an opengl error");
}
void Shader::set(const std::string& name, const VecNf<4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC4), "Mismatched argument type");
  glUniform4fv(desc.location, 1, arg.data());
}

void Shader::set(const std::string& name, const MatNf<4, 4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4), "Mismatched argument type");
  glUniformMatrix4fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<4, 3>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4x3), "Mismatched argument type");
  glUniformMatrix4x3fv(desc.location, 1, false, arg.data());
}
void Shader::set(const std::string& name, const MatNf<2, 2>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2), "Mismatched argument type");
  glUniformMatrix2fv(desc.location, 1, false, arg.data());
}
}  // namespace jcc
