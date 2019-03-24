#include "logging/log.hh"

namespace jcc {
std::ostream& operator<<(std::ostream& os, const Format& fmt) {
  if (fmt.control != Control::DO_NOTHING) {
    os << "\033[" << static_cast<int>(fmt.control) << "m";
  }

  if (fmt.foreground != ForegroundColor::DO_NOTHING &&
      fmt.background != BackgroundColor::DO_NOTHING) {
    os << "\033[" <<                                //
        static_cast<int>(fmt.foreground) << ";" <<  //
        static_cast<int>(fmt.background) << "m";
  } else {
    if (fmt.foreground != ForegroundColor::DO_NOTHING) {
      os << "\033[" << static_cast<int>(fmt.foreground) << "m";
    }
    if (fmt.background != BackgroundColor::DO_NOTHING) {
      os << "\033[" << static_cast<int>(fmt.background) << "m";
    }
  }

  return os;
}

}  // namespace jcc
