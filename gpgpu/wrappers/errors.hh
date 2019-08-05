#pragma once

#include <CL/cl.h>

namespace jcc {

// TODO: Make a macro for this that reports line number
void check_status(cl_int status);

}  // namespace jcc