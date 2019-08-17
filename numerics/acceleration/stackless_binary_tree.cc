#include <cstddef>

#include <iostream>

namespace jcc {
// void traverse1(int[] xtree, int n) {
//   uint32_t level_start = 1u;
//   uint32_t level_index = 0u;
//   while (level_start <= 1u) {
//     int node_ind = level_start + level_index - 1;
//     std::cout << "Visiting: " << xtree[node_ind] << std::endl;
//   }
// }

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

void traverse(int xtree[], int n) {
  int next = 0;
  bool backtracking_from_left = false;
  bool backtracking_from_right = false;

  int n_visited = 0;

  while (n_visited < n) {
    const int cur_node = next;

    if (!(backtracking_from_right || backtracking_from_left)) {
      n_visited++;
      std::cout << "Visiting: " << cur_node << std::endl;
    }

    if (backtracking_from_left) {
      if (rchild(cur_node) < n) {
        next = rchild(cur_node);
        backtracking_from_right = false;
        backtracking_from_left = false;
      } else {
        backtracking_from_right = !is_left(cur_node);
        backtracking_from_left = is_left(cur_node);
        next = parent(cur_node);
      }
    } else if (backtracking_from_right) {
      backtracking_from_right = !is_left(cur_node);
      backtracking_from_left = is_left(cur_node);

      next = parent(cur_node);
    } else if (lchild(cur_node) >= n) {
      backtracking_from_right = !is_left(cur_node);
      backtracking_from_left = is_left(cur_node);
      next = parent(cur_node);
    } else if (lchild(cur_node) < n) {
      backtracking_from_right = false;
      backtracking_from_left = false;
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