#pragma once

#include "eigen.hh"

struct clFluidSimConfig {
  cl_float nu;
  cl_float dt_sec;
  cl_float dx_m;
  cl_int render_mode;
  cl_int max_iteration;
  cl_int debug_mode;
};

struct FluidSimConfig {
  float nu = 0.8;
  float dt_sec = 0.01;
  float dx_m = 0.1;
  int render_mode = 0;
  int max_iteration = 0;
  int debug_mode = 0;
  clFluidSimConfig convert() const {
    clFluidSimConfig cvt;
    cvt.nu = nu;
    cvt.dt_sec = dt_sec;
    cvt.dx_m = dx_m;
    cvt.render_mode = render_mode;
    cvt.max_iteration = max_iteration;
    cvt.debug_mode = debug_mode;
    return cvt;
  }
};

