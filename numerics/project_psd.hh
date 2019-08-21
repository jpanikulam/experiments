#pragma once

#include "eigen.hh"

namespace numerics {

template <int N>
MatNd<N, N> project_psd(const MatNd<N, N>& mat, double min_eigenvalue = 0.2) {
  using Mat = MatNd<N, N>;
  const Eigen::SelfAdjointEigenSolver<Mat> solver(mat);
  const VecNd<N> eigenvalues = solver.eigenvalues();

  const VecNd<N> clamped_eigenvalues = eigenvalues.cwiseMax(min_eigenvalue);

  const Mat reconstructed = solver.eigenvectors() * clamped_eigenvalues.asDiagonal() *
                            solver.eigenvectors().inverse();
  return reconstructed;
}

template <int N>
MatNd<N, N> deproject_psd(const MatNd<N, N>& mat, double max_eigenvalue = 100.0) {
  using Mat = MatNd<N, N>;
  const Eigen::SelfAdjointEigenSolver<Mat> solver(mat);
  const VecNd<N> eigenvalues = solver.eigenvalues();

  const VecNd<N> clamped_eigenvalues = eigenvalues.cwiseMin(max_eigenvalue);

  const Mat reconstructed = solver.eigenvectors() * clamped_eigenvalues.asDiagonal() *
                            solver.eigenvectors().inverse();
  return reconstructed;
}

}  // namespace numerics