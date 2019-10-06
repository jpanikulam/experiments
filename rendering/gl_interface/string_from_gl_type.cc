#include "rendering/gl_interface/string_from_gl_type.hh"

namespace jcc {

std::string string_from_gl_type(const GLenum type) {
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
    case GL_SAMPLER_2D:
      return "Sampler2D";
    case GL_SAMPLER_3D:
      return "Sampler3D";
    default:
      return "Unknown";
  }
}
}  // namespace jcc