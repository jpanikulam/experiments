#include "eigen.hh"

//%deps(yaml-cpp)
#include <yaml-cpp/yaml.h>
#include <cassert>

namespace jcc {

template <int rows, int cols>
void set_matrix(YAML::Node& node, const std::string& id, const MatNd<rows, cols>& mat) {
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      node[id].push_back(mat(i, j));
    }
  }
}

template <int rows, int cols>
MatNd<rows, cols> read_matrix(const YAML::Node& node) {
  using Mat = const MatNd<rows, cols>;
  assert(node);
  const std::vector<double> vv = node.as<std::vector<double>>();
  const Eigen::Map<Mat> mmap_mat(vv.data());
  // Copy the map
  const Mat mat = mmap_mat;
  return mat;
}

template <int rows, int cols>
void read_matrix(const YAML::Node& node, MatNd<rows, cols>& mat) {
  using Mat = const MatNd<rows, cols>;
  assert(node);
  const std::vector<double> vv = node.as<std::vector<double>>();
  const Eigen::Map<Mat> mmap_mat(vv.data());
  // Copy the map
  mat = mmap_mat;
}

inline void se3_to_yaml(YAML::Node& parent_yaml, const SE3& source_from_destination) {
  set_matrix(parent_yaml, "log_R_source_from_destination",
             source_from_destination.so3().log());
  set_matrix(parent_yaml, "translation_source_from_destination",
             source_from_destination.translation());
}

inline SE3 yaml_to_se3(const YAML::Node& parent) {
  const SO3 R_source_from_destination =
      SO3::exp(read_matrix<3, 1>(parent["log_R_source_from_destination"]));
  const jcc::Vec3 translation_source_from_destination =
      read_matrix<3, 1>(parent["translation_source_from_destination"]);
  const SE3 source_from_destination =
      SE3(R_source_from_destination, translation_source_from_destination);
  return source_from_destination;
}

}  // namespace jcc