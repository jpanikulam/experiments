#include <vector>

#include "eigen.hh"

namespace numerics {

// Compute an orthonormal basis for a list of vectors
// Returns a vector of the same or smaller length than `vectors`, which is
// the list of basis vectors
//
//
// This could be used for sparse QR
// TODO: Implement blocks-sparse QR
template <int DIM>
std::vector<VecNd<DIM>> gram_schmidt(const std::vector<VecNd<DIM>>& vectors) {
  const int num_vecs = static_cast<int>(vectors.size());

  std::vector<VecNd<DIM>> basis;
  basis.reserve(num_vecs);

  constexpr double MIN_SQ_NORM = 1e-6;
  for (int k = 0; k < num_vecs; ++k) {
    VecNd<DIM> proj = vectors[k];
    for (int i = 0; i < basis.size(); ++i) {
      proj -= (vectors[k].dot(basis[i]));
    }

    // If the current vector has no new information, don't add it
    const double proj_norm_sq = proj.squaredNorm();
    if (proj_norm_sq < MIN_SQ_NORM) {
      continue;
    } else {
      basis.push_back(proj / std::sqrt(proj_norm_sq));
    }

    // Stop if we have as many basis vectors as the dimension of the ambient space
    if (static_cast<int>(basis.size()) >= DIM) {
      break;
    }
  }

  return basis;
}

}  // namespace numerics
