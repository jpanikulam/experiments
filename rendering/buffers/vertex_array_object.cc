#include <GL/glew.h>

#include "rendering/buffers/vertex_array_object.hh"

#include "logging/assert.hh"

namespace jcc {

VertexArrayObject::VertexArrayObject(
    const std::map<std::string, AttributeDescription>& desc)
    : attribute_from_name_(desc) {
  glGenVertexArrays(1, &vao_);
}

VertexArrayObject::~VertexArrayObject() {
  if (vao_ != 0u) {
    destroy();
  }
}

void VertexArrayObject::bind() const {
  glBindVertexArray(vao_);
}

void VertexArrayObject::destroy() {
  glDeleteBuffers(allocated_buffers_.size(), allocated_buffers_.data());
  glDeleteVertexArrays(1, &vao_);
  allocated_buffers_.clear();
  vao_ = 0u;
}

void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<2, 3>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2x3), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 6 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<2, 4>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2x4), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 8 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<3, 2>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3x2), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 6 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<3, 4>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3x4), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 12 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<4, 2>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4x2), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 8 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<3, 3>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT3), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 9 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name, const std::vector<VecNf<2>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC2), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 2 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name, const std::vector<VecNf<3>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC3), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 3 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name, const std::vector<VecNf<4>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_VEC4), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 4 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name, const std::vector<float>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 1 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<4, 4>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 16 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<4, 3>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT4x3), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 12 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
void VertexArrayObject::set(const std::string& name,
                            const std::vector<MatNf<2, 2>>& arg) {
  const std::string err_str = name + " was not available";
  if (debug_mode_) {
    if (0u == attribute_from_name_.count(name)) {
      // jcc::Warning() << "Not using " << name << " in shader" << std::endl;
      return;
    }
  } else {
    JASSERT_EQ(attribute_from_name_.count(name), 1u, err_str.c_str());
  }

  const auto& desc = attribute_from_name_.at(name);
  JASSERT_EQ(desc.type, static_cast<int>(GL_FLOAT_MAT2), "Mismatched argument type");
  glBindVertexArray(vao_);
  GLuint vbo_id;
  glGenBuffers(1, &vbo_id);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
  glBufferData(GL_ARRAY_BUFFER, arg.size() * 4 * sizeof(GLfloat), arg.data(),
               GL_STATIC_DRAW);
  constexpr GLint SIZE = 3;
  constexpr bool NORMALIZED = false;
  constexpr int STRIDE = 0;
  glVertexAttribPointer(desc.location, SIZE, GL_FLOAT, NORMALIZED, STRIDE, 0);
  glEnableVertexAttribArray(desc.location);
  allocated_buffers_.push_back(vbo_id);
}
}  // namespace jcc