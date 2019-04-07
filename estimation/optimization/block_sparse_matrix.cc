#include "estimation/optimization/block_sparse_matrix.hh"

#include "logging/assert.hh"
#include "numerics/is_pd.hh"
#include "out.hh"

namespace estimation {
namespace optimization {

namespace {

void insert_into_triplets(const Eigen::MatrixXd& mat,
                          int offset_row,
                          int offset_col,
                          Out<std::vector<Eigen::Triplet<double>>> triplets) {
  // triplets.reserve(mat.rows() * mat.cols());

  for (int row = 0; row < mat.rows(); ++row) {
    for (int col = 0; col < mat.cols(); ++col) {
      const double val = mat(row, col);
      if (std::abs(val) > 1e-12) {
        triplets->emplace_back(row + offset_row, col + offset_col, val);
      }
    }
  }
}

Eigen::SparseVector<double> to_sparse(const std::vector<VecXd>& v) {
  int numel = 0;
  for (const auto& sub_v : v) {
    numel += sub_v.rows();
  }

  int count_so_far = 0;
  Eigen::SparseVector<double> sp_vec(numel);
  for (const auto& sub_v : v) {
    for (int sub_row = 0; sub_row < sub_v.rows(); ++sub_row) {
      sp_vec.coeffRef(count_so_far + sub_row) = sub_v[sub_row];
    }
    count_so_far += sub_v.rows();
  }

  return sp_vec;
}

}  // namespace

BlockSparseMatrix::BlockSparseMatrix(const int n_block_rows, const int n_block_cols) {
  rows_.resize(n_block_rows);
  n_cols_.resize(n_block_cols, -1);
  valid_ = true;
}

void BlockSparseMatrix::set(int row, int col, const Eigen::MatrixXd& mat) {
  JASSERT(valid_, "Must be valid");
  //
  // Assert in col/row bounds
  //

  JASSERT_GE(row, 0, "Row must be nonnegative");
  JASSERT_GE(col, 0, "Column must be nonnegative");
  JASSERT_LT(row, static_cast<int>(rows_.size()), "Row must be within the matrix");
  JASSERT_LT(col, static_cast<int>(n_cols_.size()), "Col must be within the matrix");

  //
  // Update (or assert) row
  //

  if (rows_[row].cols.empty()) {
    rows_[row].n_rows = mat.rows();
  } else {
    JASSERT_EQ(
        mat.rows(),
        rows_[row].n_rows,
        "Must have the same number of rows as the other elements in this block row");
  }

  //
  // Update (or assert) n_cols
  //

  if (n_cols_[col] == -1) {
    n_cols_[col] = mat.cols();
  } else {
    JASSERT_EQ(
        n_cols_[col],
        mat.cols(),
        "Must have the same number of columns as the other elements in this block col");
  }

  //
  // Actually update
  //

  rows_[row].cols[col] = {mat};
}

const Eigen::MatrixXd& BlockSparseMatrix::get(int row, int col) const {
  JASSERT(valid_, "Must be valid");
  //
  // Enforce row, col bounds
  //
  JASSERT_GE(row, 0, "Row must be nonnegative");
  JASSERT_GE(col, 0, "Column must be nonnegative");
  JASSERT_LT(row, static_cast<int>(rows_.size()), "Row must be within the matrix");
  JASSERT_LT(col, static_cast<int>(n_cols_.size()), "Col must be within the matrix");

  // This is (implicitly) zero, but you probably didn't mean to do this
  const auto& block_row = rows_.at(row);
  JASSERT_NE(block_row.cols.count(col),
             0u,
             "Cannot access an empty block (You probably didn't mean to do this)");
  return block_row.cols.at(col).block;
}

int BlockSparseMatrix::real_rows() const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  int total = 0;
  for (const auto& row : rows_) {
    JASSERT_NE(row.n_rows, -1, "Encountered an uninitialized row");
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols() const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  int total = 0;
  for (const int n : n_cols_) {
    total += n;
  }
  return total;
}

int BlockSparseMatrix::real_rows_above_block(int block_row) const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  JASSERT_GE(block_row, 0, "Block row must be greater than zero");
  JASSERT_LT(block_row, static_cast<int>(rows_.size()), "Block must be in matrix");

  int total = 0;
  for (int i = 0; i < block_row; ++i) {
    const auto& row = rows_[i];
    JASSERT_NE(row.n_rows, -1, "Row must be initialized");
    total += row.n_rows;
  }

  return total;
}

int BlockSparseMatrix::real_cols_left_of_block(int block_col) const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  JASSERT_GE(block_col, 0, "Block col must be in matrix");
  JASSERT_LT(block_col, static_cast<int>(n_cols_.size()), "Block must be in matrix");

  int total = 0;
  for (int i = 0; i < block_col; ++i) {
    total += n_cols_[i];
  }
  return total;
}

BlockSparseMatrix::SpMat BlockSparseMatrix::to_eigen_sparse() const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  const int rows = real_rows();
  const int cols = real_cols();
  SpMat mat(rows, cols);

  std::vector<Eigen::Triplet<double>> triplets;

  // (This is knowable)
  // Eigen::VectorXi nnz_cols(cols);

  const int num_rows = static_cast<int>(rows_.size());
  for (int block_row_ind = 0; block_row_ind < num_rows; ++block_row_ind) {
    const int true_row_num = real_rows_above_block(block_row_ind);
    const auto& row = rows_.at(block_row_ind);

    for (const auto& col_block : row.cols) {
      const int block_col_ind = col_block.first;
      const auto& col = col_block.second;
      const int true_col_num = real_cols_left_of_block(block_col_ind);

      // [TODO] Possibly can be reference
      const MatXd submat = col.block;

      insert_into_triplets(submat, true_row_num, true_col_num, out(triplets));
    }
  }

  mat.setFromTriplets(triplets.begin(), triplets.end());

  return mat;
}

jcc::Optional<VecXd> BlockSparseMatrix::solve_lst_sq(const std::vector<VecXd>& residuals,
                                                     const BlockSparseMatrix& R_inv,
                                                     const double lambda) const {
  JASSERT(valid_, "Block Sparse Matrix must be valid");
  const SpMat J = to_eigen_sparse();

  SpMat identity(J.cols(), J.cols());
  identity.setIdentity();

  constexpr bool LEVENBERG_ONLY = false;

  //
  // Levenberg
  //
  if constexpr (LEVENBERG_ONLY) {
    const SpMat JtRi = J.transpose() * R_inv.to_eigen_sparse();
    const SpMat damped_JtRiJ = (JtRi * J) + (lambda * identity);
    const SpVec v = to_sparse(residuals);
    const SpMat Jtv = J.transpose() * v;
    const Eigen::SimplicialCholesky<SpMat> chol(damped_JtRiJ);
    if (chol.info() != Eigen::Success) {
      std::cout << "Failed to solve: " << chol.info() << std::endl;
      return {};
    }

    const VecXd delta = chol.solve(Jtv);
    return {delta};
  }

  //
  // Levenberg-Marquardt
  //

  if constexpr (!LEVENBERG_ONLY) {
    const SpMat JtRi = J.transpose() * R_inv.to_eigen_sparse();
    const SpMat JtRiJ = (JtRi * J);
    const VecXd jtj_diag = JtRiJ.diagonal();

    SpMat sp_diag(JtRiJ.cols(), JtRiJ.cols());
    std::vector<Eigen::Triplet<double>> triplets;
    for (int k = 0; k < JtRiJ.cols(); ++k) {
      triplets.emplace_back(k, k, jtj_diag[k]);
    }
    sp_diag.setFromTriplets(triplets.begin(), triplets.end());

    const SpMat damped_JtRiJ = JtRiJ + (lambda * sp_diag) + (lambda * identity);
    const Eigen::SimplicialCholesky<SpMat> chol(damped_JtRiJ);

    if (chol.info() != Eigen::Success) {
      std::cout << "Failed to solve: " << chol.info() << std::endl;
      return {};
    }

    const SpVec v = to_sparse(residuals);
    const SpMat JtRiv = JtRi * v;
    const VecXd delta = chol.solve(JtRiv);
    return {delta};
  }
}

}  // namespace optimization
}  // namespace estimation
