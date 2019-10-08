#pragma once

#include <map>

#include <Eigen/Sparse>

#include "eigen.hh"
#include "util/optional.hh"

namespace estimation {
namespace optimization {

// BSR Matrix
class BlockSparseMatrix {
  using SpMat = Eigen::SparseMatrix<double>;
  using SpVec = Eigen::SparseVector<double>;

 public:
  BlockSparseMatrix() {
    valid_ = false;
  }
  BlockSparseMatrix(const int n_block_rows, const int n_block_cols);

  void set(int row, int col, const Eigen::MatrixXd& mat);
  const Eigen::MatrixXd& get(int row, int col) const;

  int real_rows() const;
  int real_cols() const;

  int real_rows_above_block(int block_row) const;
  int real_cols_left_of_block(int block_col) const;

  SpMat to_eigen_sparse() const;

  jcc::Optional<VecXd> solve_lst_sq(const std::vector<VecXd>& residuals,
                                    const BlockSparseMatrix& R_inv,
                                    const double lambda = 0.0) const;

  int block_rows() const {
    return rows_.size();
  }
  int block_cols() const {
    return n_cols_.size();
  }

  // Here's a little mock for what an LDLT will feel like.
  /*
  void ldlt() {
    D;
    L;

    for (int j = 0; j < i; ++j) {
      DDii = 0;
      for (int k = 0; k < j; ++k) {
        DDii += L[j, k] * D[k] * L[j, k].transpose();
      }
      D[j] = A[j, j] - DDii;

      for (int i = 0; i < n; ++i) {
        DD = 0;

        d_llt = llt(D[j]);
        for (int k = 0; k < j; ++k) {
          DD += L(i, k) * D[k] * L[j, k].transpose();
        }
        L[i, j] = d_llt.solve((A[i, j] - DD).transpose()).transpose();
      }
    }
  }
  */
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

  bool valid_ = false;
};
}  // namespace optimization
}  // namespace estimation