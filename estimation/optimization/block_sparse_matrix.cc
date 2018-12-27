#include "estimation/optimization/block_sparse_matrix.hh"

namespace estimation {
namespace optimization {

BlockSparseMatrix::BlockSparseMatrix(const int n_block_rows, const int n_block_cols) {
  rows_.resize(n_block_rows);
  n_cols_.resize(n_block_cols, -1);
}

void BlockSparseMatrix::set(int row, int col, const Eigen::MatrixXd& mat) {
  //
  // Assert in col/row bounds
  //

  assert(row >= 0);
  assert(col >= 0);
  assert(row < static_cast<int>(rows_.size()));
  assert(col < static_cast<int>(n_cols_.size()));

  //
  // Update (or assert) row
  //

  if (rows_[row].cols.empty()) {
    rows_[row].n_rows = mat.rows();
  } else {
    assert(mat.rows() == rows_[row].n_rows);
  }

  //
  // Update (or assert) n_cols
  //

  if (n_cols_[col] == -1) {
    n_cols_[col] = mat.cols();
  } else {
    assert(n_cols_[col] == mat.cols());
  }

  //
  // Actually update
  //

  rows_[row].cols[col] = {mat};
}

const Eigen::MatrixXd& BlockSparseMatrix::get(int row, int col) const {
  //
  // Enforce row, col bounds
  //
  assert(row >= 0);
  assert(col >= 0);
  assert(row < static_cast<int>(rows_.size()));
  assert(col < static_cast<int>(n_cols_.size()));

  // This is (implicitly) zero, but you probably didn't mean to do this
  const auto& block_row = rows_.at(row);
  assert(block_row.cols.count(col));
  return block_row.cols.at(col).block;
}

int BlockSparseMatrix::real_rows() const {
  int total = 0;
  for (const auto& row : rows_) {
    assert(row.n_rows != -1);
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols() const {
  int total = 0;
  for (const auto& n : n_cols_) {
    total += n;
  }
  return total;
}

int BlockSparseMatrix::real_rows_above_block(int block_row) const {
  assert(block_row >= 0);
  assert(block_row < static_cast<int>(rows_.size()));

  int total = 0;
  for (int i = 0; i < block_row; ++i) {
    const auto& row = rows_[i];
    assert(row.n_rows != -1);
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols_left_of_block(int block_col) const {
  assert(block_row >= 0);
  assert(block_row < static_cast<int>(rows_.size()));

  int total = 0;
  // for (const auto& n : n_cols_) {
  for (int i = 0; i < block_col; ++i) {
    total += n;
  }
  return total;
}

BlockSparseMatrix::SpMat BlockSparseMatrix::to_eigen_sparse() const {
  const int rows = real_rows();
  const int cols = real_cols();
  SpMat mat(rows, cols);

  // (This is knowable)
  // Eigen::VectorXi nnz_cols;

  for (const auto& row : rows_) {
    for (const auto& col : row.cols) {
      const MatXd submat = col.block;
    }
  }

  // mat.setFromTriplets(triplets);

  return mat;
}

}  // namespace optimization
}  // namespace estimation