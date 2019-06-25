#include "numerics/optimization/bracket.hh"

namespace numerics {

ShrinkingBracket update_bracket(const double x,
                                const double y,
                                const ShrinkingBracket& bracket) {
  JASSERT_BETWEEN(x, bracket.left_x, bracket.right_x, "ShrinkingBracket cannot grow");

  ShrinkingBracket new_bracket = bracket;

  // TODO handle this
  JASSERT_LT(y, bracket.left_y, "Nonconvex");
  JASSERT_LT(y, bracket.right_y, "Nonconvex");

  if (y < bracket.best_y) {
    if (x < bracket.best_x) {
      new_bracket.right_x = bracket.best_x;
      new_bracket.right_y = bracket.best_y;

      new_bracket.best_x = x;
      new_bracket.best_y = y;
    } else {
      new_bracket.left_x = bracket.best_x;
      new_bracket.left_y = bracket.best_y;

      new_bracket.best_x = x;
      new_bracket.best_y = y;
    }
  } else {
    if (x < bracket.best_x) {
      new_bracket.left_x = x;
      new_bracket.left_y = y;
    } else {
      new_bracket.right_x = x;
      new_bracket.right_y = y;
    }
  }
  return new_bracket;
}

}  // namespace numerics