#include "macros.hh"

namespace jcc {

template <typename A, typename B>
void assert_gt(const A& a,
               const B& b,
               char const* const msg,
               char const* const file_name,
               int line_num) {
  if (UNLIKELY(a <= b)) {
    std::cerr << file_name << ":" << line_num << ": Error, expected: " << a << " > " << b
              << " " << msg << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
void assert_ge(const A& a,
               const B& b,
               char const* const msg,
               char const* const file_name,
               int line_num) {
  if (UNLIKELY(a < b)) {
    std::cerr << file_name << ":" << line_num << ": Error, expected: " << a << " >= " << b
              << " " << msg << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
void assert_lt(const A& a,
               const B& b,
               char const* const msg,
               char const* const file_name,
               int line_num) {
  if (UNLIKELY(a >= b)) {
    std::cerr << file_name << ":" << line_num << ": Error, expected: " << a << " < " << b
              << " " << msg << std::endl;
    std::abort();
  }
}
template <typename A, typename B>
void assert_le(const A& a,
               const B& b,
               char const* const msg,
               char const* const file_name,
               int line_num) {
  if (UNLIKELY(a > b)) {
    std::cerr << file_name << ":" << line_num << ": Error, expected: " << a << " <= " << b
              << " " << msg << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
void assert_eq(const A& a,
               const B& b,
               char const* const msg,
               char const* const file_name,
               int line_num) {
  if (UNLIKELY(a == b)) {
    std::cerr << file_name << ":" << line_num << ": Error, expected: " << a << " == " << b
              << " " << msg << std::endl;
    std::abort();
  }
}

}  // namespace jcc

#define JASSERT_GE(a, b, msg) jcc::assert_ge(a, b, msg, __FILE__, __LINE__);
#define JASSERT_GT(a, b, msg) jcc::assert_gt(a, b, msg, __FILE__, __LINE__);
#define JASSERT_LE(a, b, msg) jcc::assert_le(a, b, msg, __FILE__, __LINE__);
#define JASSERT_LT(a, b, msg) jcc::assert_lt(a, b, msg, __FILE__, __LINE__);
#define JASSERT_EQ(a, b, msg) jcc::assert_eq(a, b, msg, __FILE__, __LINE__);
