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

  int rows;
  int cols;
};

}  // namespace estimation