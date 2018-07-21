#include "lanczos/rigid_body.hh"

#include "testing/gtest.hh"

namespace lanczos {

TEST(RigidBody, simulate_no_velocity) {
  RigidBody init;

  const auto result = simulate(init, {}, 1.0);

  EXPECT_EQ((result.body_from_world).log().norm(), 0.0);
}

TEST(RigidBody, position_vel_translates) {
  //
  // Setup
  //

  RigidBody init;
  init.positional_velocity = Vec3(1.0, 0.0, 0.0);

  //
  // Action
  //

  constexpr double TIME_STEP_SEC = 1.0;
  const auto result = simulate(init, {}, TIME_STEP_SEC);

  //
  // Verification
  //

  const Vec3 expected_translation = init.positional_velocity;
  const Vec3 actual_translation = result.body_from_world.translation();
  const Vec3 error_translation = actual_translation - expected_translation;

  const Vec3 error_rotation = result.body_from_world.so3().log();

  constexpr double EPS = 1e-6;
  EXPECT_LT(error_translation.norm(), EPS);
  EXPECT_LT(error_rotation.norm(), EPS);
}

TEST(RigidBody, rotational_vel_rotates) {
  //
  // Setup
  //

  RigidBody init;
  init.angular_velocity = Vec3(0.0, 0.0, 1.0);

  //
  // Action
  //

  const auto result = simulate(init, {}, 1.0);

  //
  // Verification
  //

  const Vec3 translation_error = result.body_from_world.translation();

  const Vec3 error_rotation =
      init.angular_velocity - result.body_from_world.so3().log();

  constexpr double EPS = 1e-6;
  EXPECT_LT(translation_error.norm(), EPS);
  EXPECT_LT(error_rotation.norm(), EPS);
}

TEST(RigidBody, rotation_is_an_offset) {
  //
  // Setup
  //

  RigidBody init;
  init.body_from_world.translation() = Vec3(1.0, 0.0, 0.0);
  init.angular_velocity = Vec3(0.0, 0.0, 1.0);

  //
  // Action
  //

  const auto result = simulate(init, {}, 1.0);

  //
  // Verification
  //

  const Vec3 translation_error =
      (result.body_from_world * init.body_from_world.inverse()).translation();

  const Vec3 error_rotation =
      init.angular_velocity - result.body_from_world.so3().log();

  constexpr double EPS = 1e-6;
  EXPECT_LT(translation_error.norm(), EPS);
  EXPECT_LT(error_rotation.norm(), EPS);
}

TEST(RigidBody, non_identity_start) {
  //
  // Setup
  //

  RigidBody init;
  init.body_from_world.translation() = Vec3(1.0, 0.0, 0.0);
  init.body_from_world.so3() = SO3::exp(Vec3(0.3, 0.0, 0.25));

  init.angular_velocity = Vec3(0.0, 0.0, 1.0);
  init.positional_velocity = Vec3(0.0, 0.0, 0.7);

  //
  // Action
  //

  const auto result = simulate(init, {}, 1.0);

  //
  // Verification
  //

  const Vec3 translation_error =
      init.positional_velocity -
      (result.body_from_world * init.body_from_world.inverse()).translation();

  const Vec3 error_rotation =
      init.angular_velocity -
      (result.body_from_world.so3() * init.body_from_world.so3().inverse())
          .log();

  constexpr double EPS = 1e-6;
  EXPECT_LT(translation_error.norm(), EPS);
  EXPECT_LT(error_rotation.norm(), EPS);
}

}  // namespace lanczos