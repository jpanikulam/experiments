
#include "out.hh"
#include "render_from_pose.hh"
#include "sophus.hh"

#include "vision/camera_model.hh"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
  // - Discover features in a simulated image
  // - Attempt to estimate the camera pose from those features
  //     - Try RANSAC
  //     - Try Gauss-Newton pose optimization
  // - Normalied Cross Correlation alignment refinement
  // - Attempt to calibration

  // Then: 3D structure estimation (Mapping)
  // Object classification
  //
}