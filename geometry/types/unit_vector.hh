#pragma once

#include "logging/assert.hh"

#include "eigen.hh"
#include "sophus.hh"

namespace geometry {

template <int N>
class UnitVector {
 public:
  using Vec = VecNd<N>;
  // We are using dependent types *partly* to be performant
  // So this is default uninitialized
  UnitVector() = default;

  explicit UnitVector(const Vec& v) {
    v_ = v.normalized();
  }

  explicit UnitVector(double x, double y, double z) {
    static_assert(N == 3, "Cannot use this constructor with anything but 3 arguments.");
    v_ = jcc::Vec3(x, y, z).normalized();
  }

  static UnitVector<2> from_angle(const double theta) {
    static_assert(N == 2);
    return UnitVector<2>::bless(VecNd<2>(std::cos(theta), std::sin(theta)));
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

  // TODO deprecate
  const Vec& vector() const {
    return v_;
  }

  const Vec& vec() const {
    return v_;
  }

  const Vec project(const Vec& v) const {
    return v_ * v_.dot(v);
  }

  double dot(const Vec& v) const {
    return v_.dot(v);
  }

  UnitVector unit_cross(const UnitVector& v) const {
    static_assert(N == 3, "Must have dimension 3");
    return UnitVector(Vec(v_.cross(v.vec())));
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

  static UnitVector UnitX() {
    return UnitVector(Vec::UnitX());
  }
  static UnitVector UnitY() {
    return UnitVector(Vec::UnitY());
  }
  static UnitVector UnitZ() {
    return UnitVector(Vec::UnitZ());
  }

 private:
  Vec v_;
};

using UnitVector3 = UnitVector<3>;
inline UnitVector3 operator*(const SE3& destination_from_source,
                             const UnitVector3& source) {
  return UnitVector3::bless(destination_from_source.so3() * source.vec());
}

using UnitVector3 = UnitVector<3>;
inline UnitVector3 operator*(const SO3& destination_from_source,
                             const UnitVector3& source) {
  return UnitVector3::bless(destination_from_source * source.vec());
}

using Unit3 = UnitVector3;
using UnitVector2 = UnitVector<2>;

}  // namespace geometry