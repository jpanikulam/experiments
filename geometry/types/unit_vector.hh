#pragma once

#include "logging/assert.hh"

#include "eigen.hh"

namespace geometry {

template <int N>
class UnitVector {
 public:
  using Vec = VecNd<N>;
  // We are using dependent types *partly* to be performance
  // So this is default uninitialized
  UnitVector() = default;

  explicit UnitVector(const Vec& v) {
    v_ = v.normalized();
  }

  static from_angle(const double theta) {
    static_assert(N == 2);
    return UnitVector::bless(VecNd<2>(std::cos(theta), std::sin(theta)));
  }


  // For now, the below "sinful" operations shall remain in cold storage
  /*
  UnitVector(const UnitVector&) = default;
  UnitVector& operator=(const Vec& v) {
    *this = normalize(v);
    return *this;
  }

  explicit operator Vec() const {
    return v_;
  }
  */

  const Vec& vector() const {
    return v_;
  }

  static UnitVector normalize(const Vec& v) {
    return bless(v.normalized());
  }

  static UnitVector check(const Vec& v) {
    constexpr double F_EPS = 1e-6;
    JASSERT_LT(std::abs(v.squaredNorm() - 1.0), F_EPS, "v must be of unit norm");
    return UnitVector(v);
  }

  static UnitVector bless(const Vec& v) {
    UnitVector vv;
    vv.v_ = v;
    return vv;
  }

 private:
  Vec v_;
};

using UnitVector3 = UnitVector<3>;
using UnitVector2 = UnitVector<2>;

}  // namespace geometry