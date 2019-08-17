#include <cstddef>

#include <iostream>

namespace jcc {

bool is_left(int n) {
  return (n % 2) == 1;
}

int parent(int n) {
  if (is_left(n)) {
    return (n - 1) / 2;
  } else {
    return (n - 2) / 2;
  }
}

int lchild(int n) {
  return (2 * n) + 1;
}

int rchild(int n) {
  return lchild(n) + 1;
}

int lr(int n) {
  return is_left(n) ? 1 : 2;
}

void traverse(int xtree[], int n) {
  int next = 0;
  int btrack_state = 0;
  int n_visited = 0;

  while (n_visited < n) {
    const int cur_node = next;

    if (btrack_state == 0) {
      n_visited++;
      std::cout << "Visiting: " << cur_node << std::endl;
    }

    if (1 == btrack_state) {
      if (rchild(cur_node) < n) {
        next = rchild(cur_node);
        btrack_state = 0;
      } else {
        btrack_state = lr(cur_node);
        next = parent(cur_node);
      }
    } else if (2 == btrack_state) {
      btrack_state = lr(cur_node);

      next = parent(cur_node);
    } else if (lchild(cur_node) >= n) {
      btrack_state = lr(cur_node);
      next = parent(cur_node);
    } else if (lchild(cur_node) < n) {
      btrack_state = 0;
      next = lchild(cur_node);
    } else {
    }
  }
}

}  // namespace jcc

/*
        0
    1          2
 3    4      5   6
7 8 9

*/

int main() {
  int xtree[10];
  xtree[0] = 0;
  xtree[1] = 1;
  xtree[2] = 2;
  xtree[3] = 3;
  xtree[4] = 4;
  xtree[5] = 5;
  xtree[6] = 6;
  xtree[7] = 7;
  xtree[8] = 8;
  xtree[9] = 9;

  jcc::traverse(xtree, 10);
}

//
// - Sparse voxel tree
//   - Multigrid voxel tree
// - Dual contouring
//   - Intersection free contouring on an octree grid
// - Volumetric billboards (too much fill rate)
// - Gigavoxels (only one object, too hard OIT)
// - Hybrid rasterized gigavoxel cubes
// - Parallax occlusion mapping
// - Froxels

// - Splats
// - "imperfect shadow maps"
// - Temporal anti-aliasing
//