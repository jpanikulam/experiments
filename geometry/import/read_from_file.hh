#pragma once

#include <array>
#include <fstream>
#include <string>

namespace geometry {

// Read something from a file
template <typename T>
T read_from_file(std::ifstream &stream) {
  T data;
  stream.read(reinterpret_cast<char *>(&data), sizeof(data));
  return data;
}

template <typename T, int N>
std::array<T, N> read_array_from_file(std::ifstream &stream) {
  std::array<T, N> data;
  for (int k = 0; k < N; ++k) {
    data[k] = read_from_file<T>(stream);
    // stream.seekg(0, std::ios::cur);
  }
  return data;
}

template <int N>
std::string read_string_from_file(std::ifstream &stream) {
  std::array<char, N> data = read_array_from_file<char, N>(stream);
  return std::string(data.begin(), data.end());
}

}  // namespace geometry