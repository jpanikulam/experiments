#include "gpgpu/import/load_kernel.hh"

#include <fstream>

#include "logging/assert.hh"

namespace jcc {
namespace {
std::string read_file(const std::string& path) {
  std::ifstream cl_file(path);

  std::string text = "";
  std::string line;
  while (std::getline(cl_file, line)) {
    text += line + "\n";
  }
  return text;
}

}  // namespace

cl::Program read_kernel(const ClInfo& cl_info, const std::string& path) {
  //
  // Read the source code and compile the kernel
  //
  constexpr bool BUILD = true;
  const std::string source = read_file(path);
  cl::Program program(cl_info.context, source, BUILD);

  //
  // Check program build status
  //
  const auto build_status = program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(cl_info.device);
  const std::string error =
      "\n" + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(cl_info.device);
  JASSERT_EQ(build_status, 0, error.c_str());

  return program;
}

}  // namespace jcc
