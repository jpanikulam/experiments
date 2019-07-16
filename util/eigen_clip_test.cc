#include "testing/gtest.hh"

#include "util/eigen_clip.hh"

namespace jcc {
TEST(ClipTest, works) {
  const jcc::Vec3 vec(1.0, 2.0, 3.0);

  const jcc::Vec3 min(0.0, 3.0, 0.9);
  const jcc::Vec3 max(0.5, 5.0, 3.0);

  const jcc::Vec3 result = eigen_clip(vec, min, max);

  // Exact equality.
  EXPECT_EQ(result.x(), 0.5);
  EXPECT_EQ(result.y(), 3.0);
  EXPECT_EQ(result.z(), 3.0);
}
}  // namespace jcc
