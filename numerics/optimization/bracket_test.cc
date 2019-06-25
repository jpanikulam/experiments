#include "numerics/optimization/bracket.hh"

#include "testing/gtest.hh"

namespace numerics {

TEST(BracketTest, shrinks_leftside) {
  const ShrinkingBracket init_brkt{
      .left_x = 0.0,    //
      .left_y = 100.0,  //
      .right_x = 1.0,
      .right_y = 50.0,  //
      .best_x = 0.5,    //
      .best_y = 25.0    //
  };

  const auto result = update_bracket(0.25, 15.0, init_brkt);

  std::cout << " \n"
            << "left_x: " << result.left_x << "\n"
            << "left_y: " << result.left_y << "\n\n"
            << "best_x: " << result.best_x << "\n"
            << "best_y: " << result.best_y << "\n\n"
            << "right_x: " << result.right_x << "\n"
            << "right_y: " << result.right_y << "\n"
            << std::endl;

  EXPECT_LT(result.right_x, init_brkt.right_x);
}

TEST(BracketTest, shrinks_leftside2) {
  const ShrinkingBracket init_brkt{
      .left_x = 0.0,    //
      .left_y = 100.0,  //
      .right_x = 1.0,
      .right_y = 100.0,  //
      .best_x = 0.5,    //
      .best_y = 25.0    //
  };

  const auto result = update_bracket(0.25, 15.0, init_brkt);

  std::cout << " \n"
            << "left_x: " << result.left_x << "\n"
            << "left_y: " << result.left_y << "\n\n"
            << "best_x: " << result.best_x << "\n"
            << "best_y: " << result.best_y << "\n\n"
            << "right_x: " << result.right_x << "\n"
            << "right_y: " << result.right_y << "\n"
            << std::endl;

  EXPECT_LT(result.right_x, init_brkt.right_x);
}

TEST(BracketTest, shrinks_rightside) {
  const ShrinkingBracket init_brkt{
      .left_x = 0.0,    //
      .left_y = 100.0,  //
      .right_x = 1.0,
      .right_y = 50.0,  //
      .best_x = 0.5,    //
      .best_y = 25.0    //
  };

  const auto result = update_bracket(0.75, 15.0, init_brkt);

  std::cout << " \n"
            << "left_x: " << result.left_x << "\n"
            << "left_y: " << result.left_y << "\n\n"
            << "best_x: " << result.best_x << "\n"
            << "best_y: " << result.best_y << "\n\n"
            << "right_x: " << result.right_x << "\n"
            << "right_y: " << result.right_y << "\n"
            << std::endl;
}

}  // namespace numerics