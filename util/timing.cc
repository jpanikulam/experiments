#include "util/timing.hh"

namespace jcc {
ScopedTimer::ScopedTimer() {
  start_time_ = jcc::now();
}
ScopedTimer::~ScopedTimer() {
  std::cout << "Timer took: " << estimation::to_seconds(jcc::now() - start_time_)
            << " Seconds" << std::endl;
}

}  // namespace jcc