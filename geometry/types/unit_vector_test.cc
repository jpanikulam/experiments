#include "geometry/types/unit_vector.hh"

#include "eigen.hh"
#include "testing/gtest.hh"

namespace geometry {

TEST(UnitVectorTest, implicit_assignment) {
  const jcc::Vec3 v0 = jcc::Vec3(1.0, 2.0, 3.0);
  UnitVector3 vv = UnitVector3::normalize(v0);
  EXPECT_DOUBLE_EQ(vv.vector().norm(), 1.0);

  const jcc::Vec3 v1 = vv.vector();
  EXPECT_DOUBLE_EQ(v1.norm(), 1.0);
}

TEST(UnitVectorTest, bless) {
  const jcc::Vec3 v0 = jcc::Vec3(1.0, 2.0, 3.0);
  const UnitVector3 vv = UnitVector3::bless(v0);

  // Contract: v0 is exactly unchanged
  EXPECT_EQ((vv.vector() - v0).norm(), 0.0);
}

}  // namespace geometry