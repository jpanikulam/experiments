#include "numerics/acceleration/segment_tree.hh"

#include "testing/gtest.hh"

namespace jcc {

TEST(SegmentTree, segments) {
  std::vector<Interval<int>> intervals = {
      {0, 1},
      {1, 2},
      {3, 9},
  };
  SegmentTree<int> segtree(intervals);
}

TEST(SegmentTree, bst) {
  const std::vector<int> vv = {1, 2, 3, 7, 10, 13, 19, 21};
  const auto result = build_bst(vv);

  for (const auto i : result) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;
}

}  // namespace jcc
