#pragma once

#include <Eigen/StdVector>
#include <Eigen/Dense>

template <int ROWS>
using VecNd = Eigen::Matrix<double, ROWS, 1>;

template <int ROWS, int COLS>
using MatNd = Eigen::Matrix<double, ROWS, COLS>;

template <int ROWS>
using SquareMatNd = Eigen::Matrix<double, ROWS, ROWS>;


namespace jcc {
using Vec1 = VecNd<1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
}  // namespace jcc
