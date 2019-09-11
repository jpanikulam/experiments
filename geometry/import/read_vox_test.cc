#include "geometry/import/read_vox.hh"

#include "testing/gtest.hh"

namespace geometry {

TEST(ReadVoxTest, read_vox) {
  const std::string file_path =
      "/home/jacob/repos/experiments/rendering/assets/spaceship1.vox";
  const auto result = read_vox(file_path);

  EXPECT_TRUE(static_cast<bool>(result));
}
}  // namespace geometry
