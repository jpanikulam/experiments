#pragma once

#include "out.hh"

#include <algorithm>
#include <functional>
#include <vector>

// Max heap wrapping the `std::make_heap` functionality
template <typename T>
class Heap {
 public:
  using CompareFunction = std::function<bool(const T &a, const T &b)>;

  // Construct a max-heap with a custom comparator
  // @param cmp The comparator to use (default: std::less)
  //            Returns "true" when `a` is `less` than `b`
  Heap(const CompareFunction &cmp = [](const T &a,
                                       const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp) {
  }

  // Construct a max-heap from an existing vector
  Heap(const std::vector<T> &vector,
       const CompareFunction &cmp = [](const T &a,
                                       const T &b) { return std::less<T>()(a, b); })
      : cmp_(cmp), vector_(vector) {
    std::make_heap(vector_, cmp_);
  }

  Heap(const Heap<T> &o_heap) : cmp_(o_heap.cmp_), vector_(o_heap.vector_) {
  }

  // Remove the drop element of the heap without any copies
  void drop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    vector_.pop_back();
  }

  // Return a copy of the top (max) element of the heap, and delete it from the heap
  T pop() {
    std::pop_heap(vector_.begin(), vector_.end(), cmp_);
    const T result = vector_.back();
    vector_.pop_back();
    return result;
  }

  // Return a reference to the top (max) element of the heap
  const T &top() const {
    return vector_.front();
  }

  // Add an element to the heap
  // @param item The thing to be pushed to the heap
  void push(const T &item) {
    vector_.push_back(item);
    std::push_heap(vector_.begin(), vector_.end(), cmp_);
  }

  // Remove all elements from the heap
  void clear() {
    vector_.clear();
  }

  // @returns The number of elements in the heap
  size_t size() const {
    return vector_.size();
  }

  // @returns True if the heap is empty
  bool empty() const {
    return vector_.empty();
  }

  // Returns a sorted vector made from the elements of the heap
  std::vector<T> to_sorted_vector() const {
    Heap heap2 = *this;
    std::vector<T> result;
    while (!heap2.empty()) {
      result.push_back(heap2.top());
      heap2.drop();
    }
    return result;
  }

 private:
  CompareFunction cmp_;
  std::vector<T> vector_;
};
