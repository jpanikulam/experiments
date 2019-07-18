#pragma once

#include "eigen.hh"

#include <string>
#include <iosfwd>

namespace numerics {
std::vector<Eigen::VectorXd> csv_to_eigen(std::istream& stream);

std::vector<Eigen::VectorXd> csv_to_eigen(const std::string& path);
}  // namespace numerics
