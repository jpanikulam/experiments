#pragma once

#include <string>
namespace jcc {

class Environment {
 public:
  static std::string asset_path();
  static std::string repo_path();
};
}  // namespace jcc
