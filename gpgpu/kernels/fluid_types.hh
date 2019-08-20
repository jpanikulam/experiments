#pragma once

#include "eigen.hh"

struct clFluidSimConfig {
  cl_float nu;
  cl_float dt;
  cl_float dx;
  cl_int test_feature;
  cl_int max_iteration;
};

struct FluidSimConfig {
  float nu = 0.0;
  float dt = 0.0;
  float dx = 0.0;
  bool test_feature = false;
  int max_iteration = 0;
  clFluidSimConfig convert() const {
    clFluidSimConfig cvt;
    cvt.nu = nu;
    cvt.dt = dt;
    cvt.dx = dx;
    cvt.test_feature = static_cast<cl_int>(test_feature);
    cvt.max_iteration = max_iteration;
    return cvt;
  }
};

