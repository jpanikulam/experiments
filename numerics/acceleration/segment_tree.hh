#pragma once

#include <algorithm>
#include <vector>

// TODO
#include <iostream>

namespace jcc {

template <typename T>
struct Interval {
  T start;
  T end;

  bool intersects(const Interval<T>& other) {
    if (start > other.end || end < other.start) {
      return false;
    }
    return true;
  }
};

inline int left_child(int i) {
  return (2 * i) + 1;
}
inline int right_child(int i) {
  return left_child(i) + 1;
}

template <typename T>
void build_bst(const std::vector<T>& sorted_inputs,
               std::vector<T>& result,
               int begin,
               int end,
               int node_ind) {
  if (node_ind >= static_cast<int>(result.size())) {
    std::cout << "    X1 [" << begin << ", " << end << "] :: " << node_ind << std::endl;
    return;
  }

  //if (begin == end) {
  //  std::cout << "    X2 [" << begin << ", " << end << "] :: " << node_ind << std::endl;
  //  return;
  //}

  const int midpoint = (begin + end) / 2;
  result[node_ind] = sorted_inputs[midpoint];

  std::cout << "[" << begin << ", " << end << "] <- " << midpoint << std::endl;
  std::cout << "  " << sorted_inputs[midpoint] << " :-> " << node_ind << std::endl;

  build_bst(sorted_inputs, result, begin, midpoint, left_child(node_ind));

  // if (midpoint + 1 <= end) {
  build_bst(sorted_inputs, result, midpoint + 1, end, right_child(node_ind));
  // }
}

template <typename T>
std::vector<T> build_bst(const std::vector<T>& sorted_inputs) {
  std::vector<T> result(sorted_inputs.size() + 1, -1);
  build_bst(sorted_inputs, result, 0, sorted_inputs.size() - 1, 0);
  return result;
}

template <typename T>
class SegmentTree {
 public:
  SegmentTree(const std::vector<Interval<T>>& intervals) : intervals_(intervals) {
    std::vector<T> endpoints;
    for (std::size_t k = 0; k < intervals.size(); ++k) {
      endpoints.push_back(intervals[k].start);
      endpoints.push_back(intervals[k].end);
    }

    std::sort(endpoints.begin(), endpoints.end());
    // L-child : 2i + 1
    // R-child : 2i + 2
    // Parent : std::floor((i - 1) / 2)

    for (std::size_t k = 0; k < endpoints.size(); ++k) {
      // if (intervals[k].start <) }
    }
  }

 private:
  void build() {
  }

  std::vector<Interval<T>> intervals_;
};

}  // namespace jcc