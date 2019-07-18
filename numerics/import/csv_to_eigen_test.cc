#include "numerics/import/csv_to_eigen.hh"

#include "logging/assert.hh"
#include "testing/gtest.hh"

#include <sstream>

namespace numerics {
TEST(CsvToEigenTest, istream_one_line) {
  // Setup
  std::stringstream ss;
  ss << "1, 2.0, 3,4.53,  0.55  , 6,7";

  // Test
  const auto result = csv_to_eigen(ss);

  // Verification
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0][0], 1);
  EXPECT_DOUBLE_EQ(result[0][1], 2.0);
  EXPECT_DOUBLE_EQ(result[0][2], 3);
  EXPECT_DOUBLE_EQ(result[0][3], 4.53);
  EXPECT_DOUBLE_EQ(result[0][4], 0.55);
  EXPECT_DOUBLE_EQ(result[0][5], 6);
  EXPECT_DOUBLE_EQ(result[0][6], 7);
}

TEST(CsvToEigenTest, istream_multiple_lines) {
  // Setup
  std::stringstream ss;
  ss << "1, 2.0, 3,4.53,  0.55  , 6,7\n";
  ss << "4.1, 2.0, 3,4.53,  0.55  , 6,7\n";
  ss << "6.1, 2.0, 3,4.53,  0.55  , 6,7\n";

  // Test
  const auto result = csv_to_eigen(ss);

  // Verification
  ASSERT_EQ(result.size(), 3u);
  EXPECT_DOUBLE_EQ(result[0][0], 1);
  EXPECT_DOUBLE_EQ(result[1][0], 4.1);
  EXPECT_DOUBLE_EQ(result[2][0], 6.1);
}

TEST(CsvToEigenTest, istream_trailing_comma) {
  // Setup
  std::stringstream ss;
  ss << "1, 2.0, 3,4.53,  0.55  , 6,7,";

  // Test
  const auto result = csv_to_eigen(ss);

  // Verification
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0][0], 1);
  EXPECT_DOUBLE_EQ(result[0][1], 2.0);
  EXPECT_DOUBLE_EQ(result[0][2], 3);
  EXPECT_DOUBLE_EQ(result[0][3], 4.53);
  EXPECT_DOUBLE_EQ(result[0][4], 0.55);
  EXPECT_DOUBLE_EQ(result[0][5], 6);
  EXPECT_DOUBLE_EQ(result[0][6], 7);
}

TEST(CsvToEigenTest, trailing_decimal) {
  // Setup
  std::stringstream ss;
  ss << "1, 2.0, 3,4.53,  0.55  , 6,7.";

  // Test
  const auto result = csv_to_eigen(ss);

  // Verification
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0][6], 7.0);
}

TEST(CsvToEigenTest, exponent) {
  // Setup
  std::stringstream ss;
  ss << "1e3, 2.0, 3,4.53,  0.55  , 6,2e7";

  // Test
  const auto result = csv_to_eigen(ss);

  // Verification
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0][0], 1e3);
  EXPECT_DOUBLE_EQ(result[0][6], 2e7);
}

TEST(CsvToEigenTest, istream_leading_comma) {
  // Setup
  std::stringstream ss;
  ss << ",1, 2.0, 3,4.53,  0.55  , 6,7,";

  // Test / Verification
  EXPECT_THROW(csv_to_eigen(ss), jcc::JccException);
}

}  // namespace numerics