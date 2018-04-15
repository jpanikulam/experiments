#pragma once

#include <eigen3/Eigen/Dense>

#include <vector>

namespace clustering {
using HashInt = uint32_t;

std::vector<HashInt> spatial_hash(const std::vector<Eigen::Vector2d> &points, double scale);
}
