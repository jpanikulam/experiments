#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

template <int ROWS>
using VecNd = Eigen::Matrix<double, ROWS, 1>;

template <int ROWS, int COLS>
using MatNd = Eigen::Matrix<double, ROWS, COLS>;

template <int ROWS>
using VecNf = Eigen::Matrix<float, ROWS, 1>;

template <int ROWS, int COLS>
using MatNf = Eigen::Matrix<float, ROWS, COLS>;

using VecXd = Eigen::VectorXd;
using MatXd = Eigen::MatrixXd;

using VecXf = Eigen::VectorXf;
using MatXf = Eigen::MatrixXf;

template <int ROWS>
using SquareMatNd = Eigen::Matrix<double, ROWS, ROWS>;

template <typename Eig>
using StdVector = std::vector<Eig, Eigen::aligned_allocator<Eig>>;

namespace jcc {
using Vec1 = VecNd<1>;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec5 = VecNd<5>;
using Vec6 = VecNd<6>;

using Vec1f = VecNf<1>;
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Vec5f = VecNf<5>;
using Vec6f = VecNf<6>;

using Vec2i = Eigen::Matrix<int, 2, 1>;
using Vec3i = Eigen::Matrix<int, 3, 1>;
using Vec4i = Eigen::Matrix<int, 4, 1>;

}  // namespace jcc
