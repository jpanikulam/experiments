#pragma once

#include <libgen.h>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "logging/log.hh"
#include "macros.hh"

namespace jcc {

namespace {

inline char* filename(char const* const argument) {
  return basename(const_cast<char*>(argument));
}

template <typename T>
inline std::stringstream to_expr(const T& val, char const* const expr) {
  std::stringstream ss;
  ss << expr << " (" << val << ")";
  return ss;
}

template <typename A, typename B>
inline std::stringstream binary_expector_string(const A& a,
                                                const B& b,
                                                char const* const a_expr,
                                                char const* const b_expr,
                                                char const* const msg,
                                                char const* const file_name,
                                                int line_num,
                                                char const* const operator_name) {
  std::stringstream ss;
  ss << Format::yellow() << filename(file_name) << ":" << line_num << ": "
     << Format::red() << "Expected" << Format::reset() << ": " << to_expr(a, a_expr).str()
     << " " << operator_name << " " << to_expr(b, b_expr).str() << ": " << msg;
  return ss;
}

}  // namespace

template <typename A, typename B>
inline void assert_gt(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a <= b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, ">")
                     .str()
              << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
inline void assert_ge(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a < b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, ">=")
                     .str()
              << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
inline void assert_lt(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a >= b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, "<")
                     .str()
              << std::endl;
    std::abort();
  }
}
template <typename A, typename B>
inline void assert_le(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a > b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, "<=")
                     .str()
              << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
inline void assert_eq(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a != b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, "==")
                     .str()
              << std::endl;

    std::abort();
  }
}

template <typename A, typename B>
inline void assert_ne(const A& a,
                      const B& b,
                      char const* const a_expr,
                      char const* const b_expr,
                      char const* const msg,
                      char const* const file_name,
                      int line_num) {
  if (UNLIKELY(a == b)) {
    std::cerr << binary_expector_string(
                     a, b, a_expr, b_expr, msg, file_name, line_num, "!=")
                     .str()
              << std::endl;
    std::abort();
  }
}

template <typename A, typename B>
inline void assert_between(const A& a,
                           const B& b,
                           const B& c,
                           char const* const a_expr,
                           char const* const b_expr,
                           char const* const c_expr,
                           char const* const msg,
                           char const* const file_name,
                           int line_num) {
  if (UNLIKELY(((a < b) || (a > c)))) {
    std::cerr << Format::yellow() << filename(file_name) << ":" << line_num << ": "
              << Format::red() << "Expected" << Format::reset() << ": "
              << to_expr(b, b_expr).str() << " <= " << to_expr(a, a_expr).str()
              << " <= " << to_expr(c, c_expr).str() << ": " << msg;

    std::abort();
  }
}

template <typename A, typename B>
inline void assert_outside(const A& a,
                           const B& b,
                           const B& c,
                           char const* const a_expr,
                           char const* const b_expr,
                           char const* const c_expr,
                           char const* const msg,
                           char const* const file_name,
                           int line_num) {
  if (UNLIKELY(((a < b) || (a > c)))) {
    std::cerr << Format::yellow() << filename(file_name) << ":" << line_num << ": "
              << Format::red() << "Expected" << Format::reset() << ": "
              << to_expr(b, b_expr).str() << " < " << to_expr(a, a_expr).str() << " < "
              << to_expr(c, c_expr).str() << ": " << msg;

    std::abort();
  }
}

inline void assert_true(const bool& a,
                        char const* const a_expr,
                        char const* const msg,
                        char const* const file_name,
                        int line_num) {
  if (UNLIKELY(!a)) {
    std::cerr << Format::yellow() << filename(file_name) << ":" << line_num << ": "
              << Format::red() << "Expected" << Format::reset() << ": " << a_expr
              << std::boolalpha << " " << a << ": " << msg << std::endl;

    std::abort();
  }
}

inline void assert_false(const bool& a,
                         char const* const a_expr,
                         char const* const msg,
                         char const* const file_name,
                         int line_num) {
  if (UNLIKELY(a)) {
    std::cerr << Format::yellow() << filename(file_name) << ":" << line_num << ": "
              << Format::red() << "Expected" << Format::reset() << ": " << a_expr
              << std::boolalpha << " " << a << ": " << msg << std::endl;

    std::abort();
  }
}

}  // namespace jcc

//
// Comparison Assertions
//

#define JASSERT_GE(a, b, msg) jcc::assert_ge(a, b, #a, #b, msg, __FILE__, __LINE__);
#define JASSERT_GT(a, b, msg) jcc::assert_gt(a, b, #a, #b, msg, __FILE__, __LINE__);
#define JASSERT_LE(a, b, msg) jcc::assert_le(a, b, #a, #b, msg, __FILE__, __LINE__);
#define JASSERT_LT(a, b, msg) jcc::assert_lt(a, b, #a, #b, msg, __FILE__, __LINE__);
#define JASSERT_EQ(a, b, msg) jcc::assert_eq(a, b, #a, #b, msg, __FILE__, __LINE__);
#define JASSERT_NE(a, b, msg) jcc::assert_ne(a, b, #a, #b, msg, __FILE__, __LINE__);

//
// Ternary Assertions
//

#define JASSERT_BETWEEN(a, b, c, msg) \
  jcc::assert_between(a, b, c, #a, #b, #c, msg, __FILE__, __LINE__);
#define JASSERT_OUTSIDE(a, b, c, msg) \
  jcc::assert_outside(a, b, c, #a, #b, #c, msg, __FILE__, __LINE__);

//
// Unary Assertions
//

#define JASSERT(a, msg) jcc::assert_true(a, #a, msg, __FILE__, __LINE__);
#define JASSERT_TRUE(a, msg) jcc::assert_true(a, #a, msg, __FILE__, __LINE__);
#define JASSERT_FALSE(a, msg) jcc::assert_false(a, #a, msg, __FILE__, __LINE__);
