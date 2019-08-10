#pragma once

#include "logging/assert.hh"

#include <CL/cl.h>

#include <string>

namespace jcc {

std::string error_name(const cl_int error);

}  // namespace jcc
#define JCHECK_STATUS(a) JASSERT_EQ(a, 0, jcc::error_name(a).c_str());