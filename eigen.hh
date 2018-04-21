#pragma once

#include <Eigen/Dense>

template <int ROWS>
using VecNd = Eigen::Matrix<double, ROWS, 1>;

template <int ROWS, int COLS>
using MatNd = Eigen::Matrix<double, ROWS, COLS>;

template <int ROWS>
using SquareMatNd = Eigen::Matrix<double, ROWS, ROWS>;
