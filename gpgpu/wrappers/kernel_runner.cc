#include "gpgpu/wrappers/kernel_runner.hh"

#include "util/ascii/ascii_histogram.hh"

#include <iostream>

namespace jcc {

KernelRunner::KernelRunner(const cl::CommandQueue& cmd_queue,
                           const WorkGroupConfig& work_group_cfg,
                           const bool enable_profiling) {
  cmd_queue_ = cmd_queue;
  enable_profiling_ = enable_profiling;
  work_group_cfg_ = work_group_cfg;
}

void KernelRunner::run(const cl::Kernel& kernel, const int profiling_period) {
  cl::Event event;
  cl_status status;

  const std::string kernel_name = kernel.getInfo<CL_KERNEL_FUNCTION_NAME>(status);
  JCHECK_STATUS(status);

  JCHECK_STATUS(cmd_queue.enqueueNDRangeKernel(
      kernel, work_group_cfg_.work_group_offset, work_group_cfg_.work_group_size,
      work_group_cfg_.local_size, nullptr, event));

  auto& profile = profiling_[kernel_name];
  if (profile.total_calls % profiling_period == 0) {
    cmd_queue.finish();
    const auto queued = estimation::from_nanoseconds(
        event.getProfilingInfo<CL_PROFILING_COMMAND_QUEUED>(&status));
    const auto submit = estimation::from_nanoseconds(
        event.getProfilingInfo<CL_PROFILING_COMMAND_SUBMIT>(&status));
    const auto start = estimation::from_nanoseconds(
        event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&status));
    const auto end = estimation::from_nanoseconds(
        event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&status));

    const TimingMeasurements time_meas{.queued = queued,  //
                                       .submit = submit,  //
                                       .start = start,    //
                                       .end = end};

    //
    // Record our measurements
    //
    profile.measurements.push_back(time_meas);
    profile.total_calls++;
  }
}

KernelRunner::print_average_execution_times() {
  for (const auto& profile_element : profiling_) {
  }
}

}  // namespace jcc