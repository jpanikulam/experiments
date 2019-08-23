#include <CL/cl.hpp>

#include "estimation/time_point.hh"

#include <map>

namespace jcc {

struct WorkGroupConfig {
  cl::NDRange work_group_size;
  cl::NDRange work_group_offset;
  cl::NDRange local_size;
}

class KernelRunner {
 public:
  KernelRunner(const cl::CommandQueue& cmd_queue,
               const WorkGroupConfig& work_group_cfg,
               const bool enable_profiling = true);

  void run(const cl::Kernel& kernel,
           const std::string& name,
           const int profiling_period == 0);

  void print_average_execution_times();

 private:
  struct TimingMeasurements {
    cl_ulong queued;
    cl_ulong submit;
    cl_ulong start;
    cl_ulong end;
  };

  struct ProfilingInfo {
    int total_calls = 0;
    std::vector<TimingMeasurements> measurements;
  };

  cl::CommandQueue cmd_queue_;
  std::map<std::string, ProfilingInfo> profiling_;
  WorkGroupConfig work_group_cfg_;

  bool enable_profiling_ = false;
};
}  // namespace jcc