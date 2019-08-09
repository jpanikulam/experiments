#pragma once

#include <CL/cl.h>

#include <string>

namespace jcc {

// TODO: Make a macro for this that reports line number
void check_status(cl_int status);
std::string error_name(const cl_int error);

}  // namespace jcc
#define JCHECK_STATUS(a) JASSERT_EQ(a, 0, jcc::error_name(a).c_str());