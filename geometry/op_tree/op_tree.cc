#include "geometry/op_tree/op_tree.hh"

namespace jcc {

namespace {
std::string operator_to_string(const CsgOperation op) {
  switch (op) {
    case CsgOperation::ADD:
      return " u ";
      break;
    case CsgOperation::SUBTRACT:
      return " \\ ";
      break;
    case CsgOperation::INTERSECT:
      return " n ";
      break;
    default:
      return "XXX";
  }
}

}  // namespace

CsgExpression CsgElement::add(const CsgElement& other) const {
  return CsgExpression(CsgOperation::ADD, *this, other);
}
CsgExpression CsgElement::subtract(const CsgElement& other) const {
  return CsgExpression(CsgOperation::SUBTRACT, *this, other);
}

CsgExpression CsgElement::intersect(const CsgElement& other) const {
  return CsgExpression(CsgOperation::INTERSECT, *this, other);
}

std::stringstream CsgExpression::gen_string() const {
  std::stringstream ss;

  const std::string left_string = static_cast<bool>(l_element_)
                                      ? l_element_->name()
                                      : "(" + l_expression_->gen_string().str() + ")";
  const std::string right_string = static_cast<bool>(r_element_)
                                       ? r_element_->name()
                                       : "(" + r_expression_->gen_string().str() + ")";
  ss << left_string << operator_to_string(operation_) << right_string;
  return ss;
}

}  // namespace jcc