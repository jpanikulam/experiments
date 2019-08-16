#include "eigen.hh"
#include "sophus.hh"

namespace jcc {

struct __attribute__((packed)) clSE3 {
  clSE3(const SE3 &se3) {
    const MatNd<3, 3> mat = se3.so3().matrix();
    r0[0] = mat(0, 0);
    r0[1] = mat(0, 1);
    r0[2] = mat(0, 2);
    r0[3] = 0.0f;

    r1[0] = mat(1, 0);
    r1[1] = mat(1, 1);
    r1[2] = mat(1, 2);
    r1[3] = 0.0f;

    r2[0] = mat(2, 0);
    r2[1] = mat(2, 1);
    r2[2] = mat(2, 2);
    r2[3] = 0.0f;

    t[0] = se3.translation().x();
    t[1] = se3.translation().y();
    t[2] = se3.translation().z();
    t[3] = 0.0f;
  }
  float r0[4];
  float r1[4];
  float r2[4];
  float t[4];
};
}  // namespace jcc