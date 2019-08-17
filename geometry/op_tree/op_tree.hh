#pragma once

//
// Strategies:
//  - Despair
//

#include <iostream>
#include <memory>
#include <sstream>

#include "util/optional.hh"

namespace jcc {

enum class CsgOperation {
  ADD = 0,       //
  SUBTRACT = 1,  //
  INTERSECT = 2  //
};

struct CsgExpression;

class CsgElement {
 public:
  CsgElement() = default;
  CsgElement(const CsgElement&) = default;
  CsgElement(const std::string& name) : name_(name) {
  }

  CsgExpression add(const CsgElement& other) const;
  CsgExpression subtract(const CsgElement& other) const;
  CsgExpression intersect(const CsgElement& other) const;

  std::string name() const {
    return name_;
  }

 private:
  std::string name_;
};

class CsgExpression {
 public:
  CsgExpression() = default;

  CsgExpression(const CsgExpression&) = default;

  CsgExpression(const CsgOperation op_type,
                const CsgElement& first,
                const CsgElement& second) {
    operation_ = op_type;
    l_element_ = std::make_unique<CsgElement>(first);
    r_element_ = std::make_unique<CsgElement>(second);
  }

  CsgExpression(const CsgOperation op_type,
                const CsgExpression& first,
                const CsgElement& second) {
    operation_ = op_type;
    l_expression_ = std::make_unique<CsgExpression>(first);
    r_element_ = std::make_unique<CsgElement>(second);
  }

  CsgExpression(const CsgOperation op_type,
                const CsgElement& first,
                const CsgExpression& second) {
    operation_ = op_type;
    l_element_ = std::make_unique<CsgElement>(first);
    r_expression_ = std::make_unique<CsgExpression>(second);
  }

  CsgExpression(const CsgOperation op_type,
                const CsgExpression& first,
                const CsgExpression& second) {
    operation_ = op_type;
    l_expression_ = std::make_unique<CsgExpression>(first);
    r_expression_ = std::make_unique<CsgExpression>(second);
  }

  CsgExpression add(const CsgElement& other) const {
    return CsgExpression(CsgOperation::ADD, *this, other);
  }

  CsgExpression subtract(const CsgElement& other) const {
    return CsgExpression(CsgOperation::SUBTRACT, *this, other);
  }

  CsgExpression intersect(const CsgElement& other) const {
    return CsgExpression(CsgOperation::INTERSECT, *this, other);
  }

  CsgExpression add(const CsgExpression& other) const {
    return CsgExpression(CsgOperation::ADD, *this, other);
  }

  CsgExpression subtract(const CsgExpression& other) const {
    return CsgExpression(CsgOperation::SUBTRACT, *this, other);
  }

  CsgExpression intersect(const CsgExpression& other) const {
    return CsgExpression(CsgOperation::INTERSECT, *this, other);
  }

  void print() const {
    std::cout << gen_string().str() << std::endl;
  }

 private:
  std::stringstream gen_string() const;
  std::shared_ptr<CsgExpression> l_expression_;
  std::shared_ptr<CsgElement> l_element_;

  std::shared_ptr<CsgExpression> r_expression_;
  std::shared_ptr<CsgElement> r_element_;

  CsgOperation operation_;
};

}  // namespace jcc