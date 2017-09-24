#pragma once

#include "out.hh"

#include <functional>
#include <algorithm>
#include <vector>

// Max heap
template <typename T>
class Heap {
public:
  using CompareFunction = std::function<bool(const T &a, const T &b)>;

  Heap(const CompareFunction &cmp = [](const T &a, const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp) {}
  Heap(const std::vector<T> &vector,
       const CompareFunction &cmp = [](const T &a, const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp)
      , vector_(vector) {
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

  size_t size() const {
    return vector_.size();
  }

  bool empty() const {
    return vector_.empty();
  }

private:
  CompareFunction cmp_;
  std::vector<T> vector_;
};
