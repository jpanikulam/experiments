#pragma once

#include "util/time_point.hh"

#include "gpgpu/opencl.hh"

#include <map>

namespace jcc {

struct WorkGroupConfig {
  cl::NDRange work_group_size;
  cl::NDRange work_group_offset;
  cl::NDRange local_size;
};

class KernelRunner {
 public:
  KernelRunner(cl::CommandQueue& cmd_queue,
               const WorkGroupConfig& work_group_cfg,
               bool enable_profiling = true);

  bool run(const cl::Kernel& kernel,
           const WorkGroupConfig& work_group_cfg,
           const int profiling_period = 1);
  void run(const cl::Kernel& kernel, bool finish = false, const int profiling_period = 1);

  void print_average_execution_times() const;
  void print_average_execution_times(const std::string& kernel_name) const;

  cl::CommandQueue& queue() {
    return cmd_queue_;
  }

 private:
  struct TimingMeasurements {
    jcc::TimePoint queued;
    jcc::TimePoint submit;
    jcc::TimePoint start;
    jcc::TimePoint end;
  };

  struct ProfilingInfo {
    int total_calls = 0;
    std::vector<TimingMeasurements> measurements;
  };

  std::map<std::string, ProfilingInfo> profiling_;

  cl::CommandQueue& cmd_queue_;
  WorkGroupConfig work_group_cfg_;
  bool enable_profiling_ = false;
};
}  // namespace jcc