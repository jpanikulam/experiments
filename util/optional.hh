#pragma once

// #include <experimental/optional>
#include <optional>

namespace jcc {
template <typename T>
using Optional = std::optional<T>;
}