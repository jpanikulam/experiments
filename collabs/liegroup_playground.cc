#include "eigen.hh"
#include "sophus.hh"

int main() {
  const SO3 b_from_a = SO3::exp(jcc::Vec3(0.0, 0.0, 1.0));

  std::cout << "X: " << (b_from_a * jcc::Vec3::UnitX()).transpose() << std::endl;
  std::cout << "Log: " << b_from_a.log().transpose() << std::endl;
}