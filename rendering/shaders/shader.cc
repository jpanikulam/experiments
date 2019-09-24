#include <GL/glew.h>

#include "logging/assert.hh"
#include "rendering/shaders/shader.hh"

namespace jcc {

std::string gl_type_name(const GLenum type) {
  switch (type) {
    case GL_FLOAT:
      return "GLfloat[1]";
    case GL_FLOAT_VEC2:
      return "GLfloat[2]";
    case GL_FLOAT_VEC3:
      return "GLfloat[3]";
    case GL_FLOAT_VEC4:
      return "GLfloat[4]";
    case GL_INT:
      return "GLint[1]";
    case GL_INT_VEC2:
      return "GLint[2]";
    case GL_INT_VEC3:
      return "GLint[3]";
    case GL_INT_VEC4:
      return "GLint[4]";
    case GL_UNSIGNED_INT:
      return "GLuint[1]";
    case GL_UNSIGNED_INT_VEC2:
      return "GLuint[2]";
    case GL_UNSIGNED_INT_VEC3:
      return "GLuint[3]";
    case GL_UNSIGNED_INT_VEC4:
      return "GLuint[4]";
    case GL_BOOL:
      return "GLboolean[1]";
    case GL_BOOL_VEC2:
      return "GLboolean[2]";
    case GL_BOOL_VEC3:
      return "GLboolean[3]";
    case GL_BOOL_VEC4:
      return "GLboolean[4]";
    case GL_FLOAT_MAT2:
      return "GLfloat[4]";
    case GL_FLOAT_MAT2x3:
      return "GLfloat[6]";
    case GL_FLOAT_MAT2x4:
      return "GLfloat[8]";
    case GL_FLOAT_MAT3:
      return "GLfloat[9]";
    case GL_FLOAT_MAT3x2:
      return "GLfloat[6]";
    case GL_FLOAT_MAT3x4:
      return "GLfloat[12]";
    case GL_FLOAT_MAT4:
      return "GLfloat[16]";
    case GL_FLOAT_MAT4x2:
      return "GLfloat[8]";
    case GL_FLOAT_MAT4x3:
      return "GLfloat[12]";
    default:
      return "Unknown";
  }
}

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

    jcc::Debug() << "Found attribute: " << str_name << ":" << std::endl;
    jcc::Debug() << "\tType:" << gl_type_name(type) << std::endl;
    jcc::Debug() << "\tSize: " << size << std::endl;
    jcc::Debug() << "\tLocation: " << location << std::endl;

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

    jcc::Debug() << "Found Uniform: " << str_name << ":" << std::endl;
    jcc::Debug() << "\tType:" << gl_type_name(type) << std::endl;
    jcc::Debug() << "\tSize: " << size << std::endl;
    jcc::Debug() << "\tLocation: " << location << std::endl;

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
void Shader::set(const std::string& name, const MatNf<2, 3>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC3), "Mismatched argument type");
  glUniform3fv(desc.location, 1, arg.data());
}
void Shader::set(const std::string& name, const VecNf<4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC4), "Mismatched argument type");
  glUniform4fv(desc.location, 1, arg.data());
}
void Shader::set(const std::string& name, const float& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(uniform_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = uniform_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT), "Mismatched argument type");
  glUniform1f(desc.location, arg);
}
void Shader::set(const std::string& name, const MatNf<4, 4>& arg) const {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == uniform_from_name_.count(name)) {
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
      jcc::Warning() << "Not using " << name << " in shader" << std::endl;
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
