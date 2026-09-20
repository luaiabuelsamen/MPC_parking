// Tiny fixed-size dense linear algebra. The state/control dimensions here are
// 5 and 2, so everything fits in registers and there is no reason to pull in a
// matrix library.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mpcpark {

template <int R, int C>
struct Mat {
  static constexpr int rows = R;
  static constexpr int cols = C;
  double d[R * C]{};

  double& operator()(int r, int c) { return d[r * C + c]; }
  double operator()(int r, int c) const { return d[r * C + c]; }
  // Vector-style access, only meaningful for column vectors.
  double& operator()(int r) { return d[r * C]; }
  double operator()(int r) const { return d[r * C]; }

  static Mat zero() { return Mat{}; }

  static Mat identity() {
    Mat m{};
    for (int i = 0; i < std::min(R, C); ++i) m(i, i) = 1.0;
    return m;
  }

  static Mat diag(const Mat<R, 1>& v) {
    Mat m{};
    for (int i = 0; i < std::min(R, C); ++i) m(i, i) = v(i);
    return m;
  }

  Mat<C, R> transpose() const {
    Mat<C, R> t{};
    for (int r = 0; r < R; ++r)
      for (int c = 0; c < C; ++c) t(c, r) = (*this)(r, c);
    return t;
  }

  double norm() const {
    double s = 0.0;
    for (int i = 0; i < R * C; ++i) s += d[i] * d[i];
    return std::sqrt(s);
  }

  double max_abs() const {
    double s = 0.0;
    for (int i = 0; i < R * C; ++i) s = std::max(s, std::fabs(d[i]));
    return s;
  }
};

template <int R, int C>
Mat<R, C> operator+(const Mat<R, C>& a, const Mat<R, C>& b) {
  Mat<R, C> m{};
  for (int i = 0; i < R * C; ++i) m.d[i] = a.d[i] + b.d[i];
  return m;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& a, const Mat<R, C>& b) {
  Mat<R, C> m{};
  for (int i = 0; i < R * C; ++i) m.d[i] = a.d[i] - b.d[i];
  return m;
}

template <int R, int C>
Mat<R, C> operator-(const Mat<R, C>& a) {
  Mat<R, C> m{};
  for (int i = 0; i < R * C; ++i) m.d[i] = -a.d[i];
  return m;
}

template <int R, int C>
Mat<R, C> operator*(double s, const Mat<R, C>& a) {
  Mat<R, C> m{};
  for (int i = 0; i < R * C; ++i) m.d[i] = s * a.d[i];
  return m;
}

template <int R, int C>
Mat<R, C> operator*(const Mat<R, C>& a, double s) {
  return s * a;
}

template <int R, int K, int C>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
  Mat<R, C> m{};
  for (int r = 0; r < R; ++r)
    for (int k = 0; k < K; ++k) {
      const double av = a(r, k);
      if (av == 0.0) continue;
      for (int c = 0; c < C; ++c) m(r, c) += av * b(k, c);
    }
  return m;
}

template <int R, int C>
Mat<R, C>& operator+=(Mat<R, C>& a, const Mat<R, C>& b) {
  for (int i = 0; i < R * C; ++i) a.d[i] += b.d[i];
  return a;
}

template <int R, int C>
Mat<R, C>& operator-=(Mat<R, C>& a, const Mat<R, C>& b) {
  for (int i = 0; i < R * C; ++i) a.d[i] -= b.d[i];
  return a;
}

template <int R>
double dot(const Mat<R, 1>& a, const Mat<R, 1>& b) {
  double s = 0.0;
  for (int i = 0; i < R; ++i) s += a(i) * b(i);
  return s;
}

// Outer product, used to build Gauss-Newton Hessian blocks from constraint
// gradients.
template <int R, int C>
Mat<R, C> outer(const Mat<R, 1>& a, const Mat<C, 1>& b) {
  Mat<R, C> m{};
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) m(r, c) = a(r) * b(c);
  return m;
}

template <int N>
Mat<N, N> symmetrize(const Mat<N, N>& a) {
  Mat<N, N> m{};
  for (int r = 0; r < N; ++r)
    for (int c = 0; c < N; ++c) m(r, c) = 0.5 * (a(r, c) + a(c, r));
  return m;
}

// Cholesky of a symmetric positive definite matrix; returns false if the
// factorisation hits a non-positive pivot (used as the PD test in the
// backward pass).
template <int N>
bool cholesky(const Mat<N, N>& a, Mat<N, N>& l) {
  l = Mat<N, N>::zero();
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j <= i; ++j) {
      double s = a(i, j);
      for (int k = 0; k < j; ++k) s -= l(i, k) * l(j, k);
      if (i == j) {
        if (s <= 1e-12) return false;
        l(i, j) = std::sqrt(s);
      } else {
        l(i, j) = s / l(j, j);
      }
    }
  }
  return true;
}

// Solve A X = B for symmetric positive definite A via its Cholesky factor.
template <int N, int C>
bool solve_spd(const Mat<N, N>& a, const Mat<N, C>& b, Mat<N, C>& x) {
  Mat<N, N> l;
  if (!cholesky(a, l)) return false;
  Mat<N, C> y{};
  for (int c = 0; c < C; ++c) {
    for (int i = 0; i < N; ++i) {
      double s = b(i, c);
      for (int k = 0; k < i; ++k) s -= l(i, k) * y(k, c);
      y(i, c) = s / l(i, i);
    }
    for (int i = N - 1; i >= 0; --i) {
      double s = y(i, c);
      for (int k = i + 1; k < N; ++k) s -= l(k, i) * x(k, c);
      x(i, c) = s / l(i, i);
    }
  }
  return true;
}

inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

}  // namespace mpcpark
