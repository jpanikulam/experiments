#include "gpgpu/wrappers/create_context.hh"

#include "logging/assert.hh"

#include <stdlib.h>

namespace jcc {

ClInfo create_context() {
  // (globally, sorry): Disable CUDA caching
  setenv("CUDA_CACHE_DISABLE", "1", 1);

  cl::Platform platform;
  std::vector<cl::Device> devices;
  platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
  JASSERT_LT(devices.size(), 2u, "This program expects only one device");
  JASSERT_GT(devices.size(), 0u, "Found no devices, probably a driver issue!");
  const cl::Context context(devices);
  return {
      .context = context,        //
      .device = devices.front()  //
  };
}

}  // namespace jcc