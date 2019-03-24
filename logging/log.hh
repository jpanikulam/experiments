#pragma once

#include <iostream>

namespace jcc {

enum class ForegroundColor {
  DO_NOTHING = -1,
  BLACK = 30,
  RED = 31,
  GREEN = 32,
  YELLOW = 33,
  BLUE = 34,
  MAGENTA = 35,
  CYAN = 36,
  WHITE = 37
};

enum class BackgroundColor {
  DO_NOTHING = -1,
  BLACK = 40,
  RED = 41,
  GREEN = 42,
  YELLOW = 43,
  BLUE = 44,
  MAGENTA = 45,
  CYAN = 46,
  WHITE = 47
};

enum class Control {
  DO_NOTHING = -1,
  RESET = 0,
  BOLD = 1,
  UNDERLINE = 4,
  INVERSE = 7,
  BOLD_OFF = 21,
  UNDERLINE_OFF = 24,
  INVERSE_OFF = 27
};

struct Format {
  ForegroundColor foreground = ForegroundColor::DO_NOTHING;
  BackgroundColor background = BackgroundColor::DO_NOTHING;
  Control control = Control::DO_NOTHING;

  static Format reset() {
    return Format(
        {ForegroundColor::DO_NOTHING, BackgroundColor::DO_NOTHING, Control::RESET});
  }

  static Format green() {
    return Format({ForegroundColor::GREEN, BackgroundColor::DO_NOTHING, Control::RESET});
  }

  static Format red() {
    return Format({ForegroundColor::RED, BackgroundColor::DO_NOTHING, Control::RESET});
  }

  static Format bold_red() {
    return Format({ForegroundColor::RED, BackgroundColor::DO_NOTHING, Control::BOLD});
  }

  static Format cyan() {
    return Format({ForegroundColor::CYAN, BackgroundColor::DO_NOTHING, Control::RESET});
  }

  static Format blue() {
    return Format({ForegroundColor::BLUE, BackgroundColor::DO_NOTHING, Control::RESET});
  }

  static Format yellow() {
    return Format({ForegroundColor::YELLOW, BackgroundColor::DO_NOTHING, Control::RESET});
  }
  friend std::ostream& operator<<(std::ostream& os, const Format& fmt);
};

class Warning {
 public:
  template <typename T>
  Warning& operator<<(T t) {
    std::cout << Format::yellow() << t << Format::reset();
    return *this;
  }
  Warning& operator<<(std::ostream& (*f)(std::ostream& o)) {
    std::cout << f;
    return *this;
  };
};

class Success {
 public:
  template <typename T>
  Success& operator<<(T t) {
    std::cout << Format::green() << t << Format::reset();
    return *this;
  }
  Success& operator<<(std::ostream& (*f)(std::ostream& o)) {
    std::cout << f;
    return *this;
  };
};

class Error {
 public:
  template <typename T>
  Error& operator<<(T t) {
    std::cout << Format::bold_red() << t << Format::reset();
    return *this;
  }
  Error& operator<<(std::ostream& (*f)(std::ostream& o)) {
    std::cout << f;
    return *this;
  };
};

class Debug {
 public:
  template <typename T>
  Debug& operator<<(T t) {
    std::cout << Format::blue() << t << Format::reset();
    return *this;
  }

  Debug& operator<<(std::ostream& (*f)(std::ostream& o)) {
    std::cout << f;
    return *this;
  };
};

}  // namespace jcc