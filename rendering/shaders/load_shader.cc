#include <GL/glew.h>

#include "logging/assert.hh"
#include "rendering/shaders/load_shader.hh"

#include <fstream>

namespace jcc {

namespace {

std::string read_file(const std::string& path) {
  std::ifstream cl_file(path);

  std::string text = "";
  std::string line;
  while (std::getline(cl_file, line)) {
    text += line + "\n";
  }
  return text;
}

GLuint load_any_shader(const std::string& path, const GLint shader_type) {
  GLuint shader_id = glCreateShader(shader_type);
  const std::string shader_code = read_file(path);
  const char* chr_str = shader_code.c_str();
  glShaderSource(shader_id, 1u, &chr_str, nullptr);

  GLint success = -1;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &success);

  if (success != 0) {
    constexpr int LOG_SIZE = 1024;
    char log[LOG_SIZE];
    glGetShaderInfoLog(shader_id, LOG_SIZE, nullptr, log);

    const std::string build_error(log);
    const std::string error_msg = "\n" + path + "\n" + build_error;

    JASSERT_EQ(success, 0, error_msg.c_str());
  }
  return shader_id;
}

}  // namespace

Shader load_shaders(const std::string& vertex_path, const std::string& shader_path) {
  const GLuint vertex_id = load_any_shader(vertex_path, GL_VERTEX_SHADER);
  const GLuint fragment_id = load_any_shader(shader_path, GL_FRAGMENT_SHADER);

  const GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vertex_id);
  glAttachShader(program_id, fragment_id);
  glLinkProgram(program_id);
  GLint link_status = -1;
  glGetProgramiv(program_id, GL_LINK_STATUS, &link_status);

  if (link_status != GL_TRUE) {
    constexpr int LOG_SIZE = 1024;
    char log[LOG_SIZE];
    glGetProgramInfoLog(program_id, LOG_SIZE, nullptr, log);

    const std::string build_error(log);
    const std::string error_msg = "\n" + shader_path + "\n" + build_error;

    JASSERT_EQ(link_status, GL_TRUE, error_msg.c_str());
  }

  glDeleteShader(vertex_id);
  glDeleteShader(fragment_id);

  // Shader shader;
  // shader.program_id = program_id;
  // return shader;
  return Shader(program_id);
}

}  // namespace jcc
