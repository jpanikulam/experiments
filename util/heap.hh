#pragma once

#include "out.hh"

#include <algorithm>
#include <vector>

//
// Non-owning heap
//

template <typename T, class Compare>
class Heap {
  Heap(std::vector<T>& vector) vector_(vector) {
  }

  void pop() {
    std::pop_heap(vector_);
    vector_.pop_back();
  }

  T pop() {
    std::pop_heap(vector_);
    const T result = vector_.back();
    vector_.pop_back();
    return T;
  }

  const T& top() {
    return vector_.front();
  }

 private:
  // Store only a reference
  std::vector<T>& vector_;
};
