#pragma once

#include "eigen.hh"

#include "rendering/shaders/attribute_description.hh"

#include <map>
#include <string>
#include <vector>

namespace jcc {

class VertexArrayObject {
 public:
  VertexArrayObject(const std::map<std::string, AttributeDescription>&);

  void bind() const;

  void set(const std::string& name, const std::vector<MatNf<2, 3>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<2, 4>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<3, 2>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<3, 4>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<4, 2>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<3, 3>>& arg) const;
  void set(const std::string& name, const std::vector<VecNf<2>>& arg) const;
  void set(const std::string& name, const std::vector<VecNf<3>>& arg) const;
  void set(const std::string& name, const std::vector<VecNf<4>>& arg) const;
  void set(const std::string& name, const std::vector<float>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<4, 4>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<4, 3>>& arg) const;
  void set(const std::string& name, const std::vector<MatNf<2, 2>>& arg) const;

 private:
  std::map<std::string, AttributeDescription> attribute_from_name_;
  int vao_ = -1;
};
}  // namespace jcc
