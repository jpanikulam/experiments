#pragma once

#include "eigen.hh"

struct clRenderVolumeConfig {
  cl_int method;
  cl_float step_size;
  cl_float max_dist;
  cl_int test_feature;
  cl_int max_iteration;
};

struct RenderVolumeConfig {
  int method = 0;
  float step_size = 0.01;
  float max_dist = 25.0;
  bool test_feature = false;
  int max_iteration = 0;
  clRenderVolumeConfig convert() const {
    clRenderVolumeConfig cvt;
    cvt.method = method;
    cvt.step_size = step_size;
    cvt.max_dist = max_dist;
    cvt.test_feature = static_cast<cl_int>(test_feature);
    cvt.max_iteration = max_iteration;
    return cvt;
  }
};

