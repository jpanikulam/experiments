#pragma once

#include "out.hh"

#include <algorithm>
#include <vector>

// Max heap
template <typename T, class Compare = std::less<T>>
class Heap {
public:
  Heap() {}
  Heap(const std::vector<T> &vector)
      : vector_(vector) {
    std::make_heap(vector_, cmp_);
  }

  void drop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    vector_.pop_back();
  }

  T pop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    const T result = vector_.back();
    vector_.pop_back();
    return result;
  }

  const T &top() {
    return vector_.front();
  }

  void push(const T &item) {
    vector_.push_back(item);
    std::push_heap(vector_.begin(), vector_.end(), cmp_);
  }

  void clear() {
    vector_.clear();
  }

private:
  // Store only a reference
  std::vector<T> vector_;
  Compare cmp_ = Compare();
};
