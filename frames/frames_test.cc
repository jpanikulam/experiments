#include "testing/gtest.hh"

#include "frames.hh"

namespace frames {

TEST(Frames, frames) {
  const Transform<FrameId::CAMERA, FrameId::BODY> camera_from_body;

  const auto a = camera_from_body.inverse() * camera_from_body;
  (void)a;
  const auto b = camera_from_body * camera_from_body.inverse();
  (void)b;

  const Transform<FrameId::BODY, FrameId::WORLD> body_from_world;

  const auto camera_from_world = camera_from_body * body_from_world;

  const auto c = camera_from_world.inverse() * camera_from_body;
  (void)c;
  const auto d = camera_from_world * body_from_world.inverse();
  (void)d;
}
}