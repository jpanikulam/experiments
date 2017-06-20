#pragma once

#include <Eigen/Dense>

// todo
#include <iostream>

namespace slam {

class CameraModel {
 public:
  using Vec2    = Eigen::Vector2d;
  using Vec3    = Eigen::Vector3d;
  using Vec4    = Eigen::Vector4d;
  using ProjMat = Eigen::Matrix<double, 3, 3>;

  CameraModel(const ProjMat& K) : K_(K){};

  //
  // @param *: use google pls
  //
  CameraModel(const double f_x, const double f_y, const double u_0, const double v_0) {
    K_.setZero();
    K_(0, 0) = f_x;
    K_(1, 1) = f_y;
    K_(0, 2) = u_0;
    K_(1, 2) = v_0;
    K_(2, 2) = 1.0;
  }

  // @param camera_point: Point in the camera frame (3d)
  // @returns The point projected into image space
  Vec2 project(const Vec3& camera_point) const {
    const Vec3 projected_h = K_ * camera_point;
    const Vec2 projected   = projected_h.head<2>() / projected_h(2);
    return projected;
  }

  // @param image_point: Point in the image frame (2d)
  // @returns ray passing through the image point
  Vec3 unproject(const Vec2& image_point) const {
    if (!inv_set_) {
      // const ProjMat ktk = K_.transpose() * K_;
      K_inv_   = Eigen::PartialPivLU<ProjMat>(K_);
      inv_set_ = true;
    }

    const Vec3 image_point_h = Vec3(image_point.x(), image_point.y(), 1.0);
    // const Vec3 soln          = K_inv_.solve(K_.transpose() * image_point_h);
    const Vec3 soln        = K_inv_.solve(image_point_h);
    const Vec3 unprojected = soln;
    return unprojected;
  }

  const ProjMat& get_k() const {
    return K_;
  }

 private:
  ProjMat K_;

  // Inversion cache
  mutable bool                         inv_set_ = false;
  mutable Eigen::PartialPivLU<ProjMat> K_inv_;
};
}
