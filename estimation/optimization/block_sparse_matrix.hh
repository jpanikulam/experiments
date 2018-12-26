#pragma once

#include <map>

#include "eigen.hh"

namespace estimation {
namespace optimization {

// BSR Matrix
class BlockSparseMatrix {
 public:
  BlockSparseMatrix(const int n_block_rows, const int n_block_cols);

  void set(int row, int col, const Eigen::MatrixXd& mat);
  const Eigen::MatrixXd& get(int row, int col) const;

 private:
  struct Block {
    Eigen::MatrixXd block;
  };

  struct Row {
    std::map<int, Block> cols;
    int n_rows = -1;
  };

  // The number of columns in each column block
  std::vector<int> n_cols_;
  // The block rows
  std::vector<Row> rows_;
};
}  // namespace optimization
}  // namespace estimation