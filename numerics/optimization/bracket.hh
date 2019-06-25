#pragma once

#include "logging/assert.hh"

namespace numerics {

struct ShrinkingBracket {
  double left_x;
  double left_y;

  double right_x;
  double right_y;

  double best_x;
  double best_y;
};

// Make the bracket gradually smaller
//
ShrinkingBracket update_bracket(const double x,
                                const double y,
                                const ShrinkingBracket& bracket);

// TODO(jpanikulam)
double quadratic_min(const ShrinkingBracket& bracket);
double exponential_halving(const ShrinkingBracket& bracket, int iteration);

}  // namespace numerics