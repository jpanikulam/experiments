#pragma once

namespace jcc {

template <typename T>
bool between(const T& val, const T& lower, const T& upper) {
  return lower < val ? (val < upper ? true : false) : false;
}

}  // namespace jcc