#include "geometry/op_tree/op_tree.hh"

#include "testing/gtest.hh"

namespace jcc {
TEST(OpTree, op_tree) {
  const CsgElement a("a");
  const CsgElement b("b");
  const CsgElement c("c");
  const CsgElement d("d");
  const CsgElement e("e");
  const CsgElement f("f");
  const CsgElement g("g");

  const auto ab = a.add(b);
  const auto cd = c.subtract(d);

  // Do we permit repeated entries in this expression tree? I claim *no*
  const auto abcd = ab.intersect(cd);

  const auto ef = e.add(f);
  const auto efg = ef.subtract(g);

  const auto abcdefg = abcd.add(efg);
  abcdefg.print();
}
}  // namespace jcc