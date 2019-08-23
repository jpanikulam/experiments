#include "gpgpu/wrappers/kernel_runner.hh"

#include "gpgpu/wrappers/errors.hh"
#include "util/ascii/ascii_histogram.hh"

#include <iostream>

namespace jcc {

KernelRunner::KernelRunner(const cl::CommandQueue& cmd_queue,
                           const WorkGroupConfig& work_group_cfg,
                           const bool enable_profiling)
    : cmd_queue_(cmd_queue),
      work_group_cfg_(work_group_cfg),
      enable_profiling_(enable_profiling) {
}

bool KernelRunner::run(const cl::Kernel& kernel,
                       const WorkGroupConfig& work_group_cfg,
                       const int profiling_period) {
  JASSERT_GT(profiling_period, 0, "Profiling period must be greater than zero");

  cl::Event event;
  cl_int status;

  const std::string kernel_name = kernel.getInfo<CL_KERNEL_FUNCTION_NAME>(&status);
  JCHECK_STATUS(status);

  auto& profile = profiling_[kernel_name];
  JCHECK_STATUS(cmd_queue_.enqueueNDRangeKernel(kernel,
                                                work_group_cfg.work_group_offset,
                                                work_group_cfg.work_group_size,
                                                work_group_cfg.local_size,
                                                nullptr,
                                                &event));

  if (enable_profiling_ && profile.total_calls % profiling_period == 0) {
    //
    // Gather profiling data
    //

    cmd_queue_.finish();
    const cl_ulong queued = event.getProfilingInfo<CL_PROFILING_COMMAND_QUEUED>(&status);
    const cl_ulong submit = event.getProfilingInfo<CL_PROFILING_COMMAND_SUBMIT>(&status);
    const cl_ulong start = event.getProfilingInfo<CL_PROFILING_COMMAND_START>(&status);
    const cl_ulong end = event.getProfilingInfo<CL_PROFILING_COMMAND_END>(&status);

    const TimingMeasurements time_meas{.queued = jcc::from_nanoseconds(queued),  //
                                       .submit = jcc::from_nanoseconds(submit),  //
                                       .start = jcc::from_nanoseconds(start),    //
                                       .end = jcc::from_nanoseconds(end)};

    //
    // Record our measurements
    //

    profile.measurements.push_back(time_meas);
    profile.total_calls++;

    // Return true if we profiled so that outer callers know not to clFinish()
    return true;
  }
  return false;
}

void KernelRunner::run(const cl::Kernel& kernel,
                       bool finish,
                       const int profiling_period) {
  bool did_profile = run(kernel, work_group_cfg_, profiling_period);
  if (!did_profile && finish) {
    cmd_queue_.finish();
  }
}

void KernelRunner::print_average_execution_times(const std::string& kernel_name) const {
  const auto& profile = profiling_.at(kernel_name);

  std::vector<double> queued_to_submit;
  queued_to_submit.reserve(profile.measurements.size());

  std::vector<double> submit_to_start;
  submit_to_start.reserve(profile.measurements.size());

  std::vector<double> start_to_end;
  start_to_end.reserve(profile.measurements.size());

  for (const auto& meas : profile.measurements) {
    const double queued_to_submit_sec = jcc::to_seconds(meas.submit - meas.queued);
    queued_to_submit.push_back(queued_to_submit_sec);
    const double submit_to_start_sec = jcc::to_seconds(meas.start - meas.submit);
    submit_to_start.push_back(submit_to_start_sec);
    const double start_to_end_sec = jcc::to_seconds(meas.end - meas.start);
    start_to_end.push_back(start_to_end_sec);
  }
}
void KernelRunner::print_average_execution_times() const {
  for (const auto& profile_element : profiling_) {
    print_average_execution_times(profile_element.first);
  }
}

}  // namespace jcc