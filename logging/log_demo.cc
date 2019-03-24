#include "logging/log.hh"

#include <iostream>

int main() {
  const jcc::Format fmt1(
      {jcc::ForegroundColor::GREEN, jcc::BackgroundColor::RED, jcc::Control::BOLD});

  std::cout << fmt1 << "This will be bold green with a red highlight"
            << jcc::Format::reset() << std::endl;

  const jcc::Format fmt2({jcc::ForegroundColor::CYAN});
  std::cout << fmt2 << "This will be cyan" << jcc::Format::reset() << std::endl;

  std::cout << "This will be white" << std::endl;

  jcc::Debug() << "This is a debug" << std::endl;
  jcc::Success() << "This is a success" << std::endl;
  jcc::Error() << "This is an error " << 2 << std::endl;
  jcc::Warning() << "This is a warning" << std::endl;
}