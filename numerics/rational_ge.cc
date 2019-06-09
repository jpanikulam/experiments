#include "logging/assert.hh"

#include <iostream>

// Swap the positions of two rows.
// Multiply a row by a non-zero scalar.
// Add to one row a scalar multiple of another.

namespace {
// I copied this from wikipedia [1]
// [1] https://en.wikipedia.org/wiki/Euclidean_algorithm#Procedure
int gcd(int a, int b) {
  while (b != 0) {
    int t = b;
    b = a % b;
    a = t;
  }
  return std::abs(a);
}
}  // namespace

class Rational {
 public:
  Rational(int a, int b) {
    if (b < 0) {
      b = -b;
      a = -a;
    }

    // This is to ensure that different denominator '0' are equal
    if (a == 0) {
      b = 1;
    }

    numerator_ = a;
    JASSERT_GT(b, 0, "Denominator must be positive");
    denominator_ = b;
    reduce();
  }

  Rational operator*(const Rational& b) const {
    return Rational(numerator_ * b.numerator_, denominator_ * b.denominator_);
  }

  Rational operator/(const Rational& b) const {
    return (*this) * b.inv();
  }

  Rational operator+(const Rational& b) const {
    const int gcf = b.denominator_ * denominator_;
    const int numerator = (b.denominator_ * numerator_) + (denominator_ * b.numerator_);
    return Rational(numerator, gcf);
  }

  Rational operator-(const Rational& b) const {
    const int gcf = b.denominator_ * denominator_;
    const int numerator = (b.denominator_ * numerator_) - (denominator_ * b.numerator_);
    return Rational(numerator, gcf);
  }

  bool operator==(const Rational& b) const {
    return (numerator_ == b.numerator_) && (denominator_ == b.denominator_);
  }

  bool operator>(const Rational& b) const {
    const Rational ratio = (*this) / b;
    return (ratio.numerator_ > ratio.denominator_);
  }

  bool operator<(const Rational& b) const {
    const Rational ratio = (*this) / b;
    return (ratio.numerator_ < ratio.denominator_);
  }

  bool operator>=(const Rational& b) const {
    const Rational ratio = (*this) / b;
    return (ratio.numerator_ >= ratio.denominator_);
  }

  bool operator<=(const Rational& b) const {
    const Rational ratio = (*this) / b;
    return (ratio.numerator_ <= ratio.denominator_);
  }

  int numerator() const {
    return numerator_;
  }

  int denominator() const {
    return denominator_;
  }

  Rational inv() const {
    return Rational(denominator_, numerator_);
  }

 private:
  // in-place reduce
  void reduce() {
    const int divisor = gcd(numerator_, denominator_);

    numerator_ /= divisor;
    denominator_ /= divisor;
  }

  int numerator_;
  int denominator_;
};

std::ostream& operator<<(std::ostream& os, const Rational& r) {
  if (0 == r.numerator()) {
    os << "(0)";
  } else if (r.numerator() == r.denominator()) {
    os << "(1)";
  } else {
    os << "(" << r.numerator() << "/" << r.denominator() << ")";
  }
  return os;
}

template <int cols>
class RationalLinearSystem {
  using RationalLinearFunction = std::array<Rational, cols>;




 private:
  std::vector<RationalLinearFunction> linear_fcns_;
};

int main() {
  std::cout << (Rational(75, 75) > Rational(4, 7)) << std::endl;
}
