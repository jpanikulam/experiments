namespace jcc {

// Implements signum
// [1] https://stackoverflow.com/a/4609795/5451259
template <typename T>
int sign(T val) {
  return (T(0) < val) - (val < T(0));
}
}  // namespace jcc
