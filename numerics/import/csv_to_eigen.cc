#include "numerics/import/csv_to_eigen.hh"

#include "logging/assert.hh"

#include <algorithm>
#include <fstream>

namespace numerics {

// TODO:
//  1,,2, 3
// ,,1, 2, 3
namespace {
Eigen::VectorXd parse_line(const std::string& line, int columns) {
  Eigen::VectorXd vec(columns);

  int cols_so_far = 0;
  std::size_t prev_break = 0;
  for (std::size_t k = 0u; k < line.size(); ++k) {
    if (line[k] == ',') {
      JASSERT_NE(k, 0u, "Malformed CSV: Cannot have a comma in the first character");
      JASSERT_GT(k, prev_break, "Malformed CSV: Cannot have ',,'");
      const std::string number = line.substr(prev_break, k - prev_break);
      prev_break = k + 1;

      vec[cols_so_far] = std::stod(number);
      cols_so_far++;
    }
  }

  //
  // Handle the final number, which may not be between commas
  //
  if (prev_break != line.size()) {
    const std::string last_num = line.substr(prev_break, line.size() - prev_break);
    vec[cols_so_far++] = std::stod(last_num);
  }
  JASSERT_EQ(cols_so_far, columns, "A row did not have the right number of columns");

  return vec;
}
}  // namespace

std::vector<Eigen::VectorXd> csv_to_eigen(std::istream& stream) {
  std::vector<Eigen::VectorXd> vecs;

  int columns = -1;
  std::string line;
  while (std::getline(stream, line)) {
    //
    // Initializing
    //
    if (columns == -1) {
      columns = static_cast<int>(std::count(line.begin(), line.end(), ','));
      if (std::isdigit(line.back())) {
        // For no trailing comma
        columns++;
      } else if (line.back() == '.') {
        // For cases like "1, 2, 3."
        columns++;
      }
    }

    //
    // Parse the actual line
    //
    const auto vec = parse_line(line, columns);
    vecs.push_back(vec);
  }

  return vecs;
}

std::vector<Eigen::VectorXd> csv_to_eigen(const std::string& path) {
  std::ifstream csv_path(path);
  return csv_to_eigen(csv_path);
}
}  // namespace numerics
