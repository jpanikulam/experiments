#include "util/timing.hh"

namespace jcc {
PrintingScopedTimer::PrintingScopedTimer(const std::string& title) {
  start_time_ = jcc::now();
  title_ = title;
}
PrintingScopedTimer::~PrintingScopedTimer() {
  std::cout << title_ << " took: " << estimation::to_seconds(jcc::now() - start_time_)
            << " Seconds" << std::endl;
}

}  // namespace jcc