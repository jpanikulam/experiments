#pragma once

#include "eigen.hh"

struct clPlane {
  cl_float3 normal;
  cl_float d;
};

struct Plane {
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float d = 0.0;
  clPlane convert() const {
    clPlane cvt;
    cvt.normal = {normal.x(), normal.y(), normal.z()};
    cvt.d = d;
    return cvt;
  }
};

struct clSphere {
  cl_float3 origin;
  cl_float r;
};

struct Sphere {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  float r = 0.0;
  clSphere convert() const {
    clSphere cvt;
    cvt.origin = {origin.x(), origin.y(), origin.z()};
    cvt.r = r;
    return cvt;
  }
};

struct clBox {
  cl_float3 origin;
  cl_float3 extents;
};

struct Box {
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  Eigen::Vector3f extents = Eigen::Vector3f::Zero();
  clBox convert() const {
    clBox cvt;
    cvt.origin = {origin.x(), origin.y(), origin.z()};
    cvt.extents = {extents.x(), extents.y(), extents.z()};
    return cvt;
  }
};

struct clRenderConfig {
  cl_int debug_mode;
  cl_int test_feature;
  cl_int terminal_iteration;
};

struct RenderConfig {
  int debug_mode = 0;
  bool test_feature = false;
  int terminal_iteration = 0;
  clRenderConfig convert() const {
    clRenderConfig cvt;
    cvt.debug_mode = debug_mode;
    cvt.test_feature = static_cast<cl_int>(test_feature);
    cvt.terminal_iteration = terminal_iteration;
    return cvt;
  }
};

