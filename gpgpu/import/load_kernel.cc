#include "gpgpu/import/load_kernel.hh"

#include <fstream>

#include "logging/assert.hh"
#include "util/environment.hh"

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

cl::Program read_program(const ClInfo& cl_info,
                         const std::string& path,
                         const std::string& include_path) {
  //
  // Read the source code and compile the kernel
  //
  constexpr bool BUILD = false;
  const std::string source = read_file(path);
  cl::Program program(cl_info.context, source, BUILD);

  const std::string demo_incl_path = jcc::Environment::repo_path() + "gpgpu/demos/";
  const std::string kernel_incl_path = jcc::Environment::repo_path() + "gpgpu/kernels/";
  const std::string flags =
      "-Werror -cl-std=CL2.0 -I " + demo_incl_path + " -I " + kernel_incl_path;
  program.build({cl_info.device}, flags.c_str());

  //
  // Check program build status
  //
  const auto build_status = program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(cl_info.device);
  const std::string error =
      "\n" + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(cl_info.device);
  JASSERT_EQ(build_status, 0, error.c_str());

  return program;
}

}  // namespace

std::map<std::string, cl::Kernel> read_kernels(const ClInfo& cl_info,
                                               const std::string& path,
                                               const std::string& include_path) {
  std::map<std::string, cl::Kernel> kernels;
  auto program = read_program(cl_info, path, include_path);

  std::vector<cl::Kernel> kernels_vec;
  program.createKernels(&kernels_vec);

  for (auto& kernel : kernels_vec) {
    std::string name;
    kernel.getInfo(CL_KERNEL_FUNCTION_NAME, &name);

    // The OpenCL driver includes an extra null-terminator in the string.
    const int last_char = name.find('\0');

    if (last_char != -1) {
      kernels[name.substr(0, last_char)] = kernel;
    } else {
      kernels[name] = kernel;
    }
  }
  return kernels;
}

}  // namespace jcc
