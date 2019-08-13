#include "util/environment.hh"

namespace jcc {

std::string Environment::asset_path() {
  return std::string(ASSET_PATH) + "/";
}

std::string Environment::repo_path() {
  return std::string(REPO_PATH) + "/";
}

}  // namespace jcc
