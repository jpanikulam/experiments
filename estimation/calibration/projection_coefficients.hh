#pragma once

#include "eigen.hh"

namespace estimation {

// NOTE: OpenCV calibration does not support shear...wtf?
//
// TODO: A tool for annotating and auto-generating to_vector & from_vector
// [0]
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
// [1]
// http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-10-camera-calibration.pdf
struct ProjectionCoefficients {
  double fx;
  double fy;
  double cx;
  double cy;

  double p1;
  double p2;

  double k1;
  double k2;
  double k3;

  static constexpr int DIM = 9;
  VecNd<DIM> to_vector() const {
    return (VecNd<DIM>() << fx, fy, cx, cy, p1, p2, k1, k2, k3).finished();
  }

  static ProjectionCoefficients from_vector(const VecNd<DIM>& vec) {
    return ProjectionCoefficients{.fx = vec(0),
                                  .fy = vec(1),
                                  .cx = vec(2),
                                  .cy = vec(3),
                                  .p1 = vec(4),
                                  .p2 = vec(5),
                                  .k1 = vec(6),
                                  .k2 = vec(7),
                                  .k3 = vec(8)};
  }
};

}  // namespace estimation