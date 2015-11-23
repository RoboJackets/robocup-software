#pragma once

/*
  C++11 constexpr versions of cmath functions needed for the FFT.
  Copyright (C) 2012 Paul Keir
  Distributed under the GNU General Public License. See license.txt for details.
*/

#include <limits> // nan

#define M_PI   3.141592653589793
#define M_PI_2 1.570796326794897
#define M_E    2.718281828459045

constexpr double tol = 0.001;

constexpr double abs_const(const double x) { return x < 0.0 ? -x : x; }

constexpr double square_const(const double x) { return x * x; }

constexpr double sqrt_helper(const double x, const double g) {
  return abs_const(g - x / g) < tol ? g : sqrt_helper(x, (g + x / g) / 2.0);
}
constexpr double sqrt_const(const double x) { return sqrt_helper(x, 1.0); }

constexpr double cube_const(const double x) { return x * x * x; }

// Based on the triple-angle formula: sin 3x = 3 sin x - 4 sin ^3 x
constexpr double sin_helper(const double x) {
  return x < tol ? x : 3 * (sin_helper(x / 3.0)) - 4 * cube_const(sin_helper(x / 3.0));
}
constexpr double sin_const(const double x) { return sin_helper(x < 0 ? -x + M_PI : x); }

//sinh 3x = 3 sinh x + 4 sinh ^3 x
constexpr double sinh_helper(const double x) {
  return x < tol ? x : 3 * (sinh_helper(x / 3.0)) + 4 * cube_const(sinh_helper(x / 3.0));
}
constexpr double sinh_const(const double x) { return x < 0 ? -sinh_helper(-x) : sinh_helper(x); }

constexpr double cos_const(const double x) { return sin_const(M_PI_2 - x); }

constexpr double cosh_const(const double x) { return sqrt_const(1.0 + square_const(sinh_const(x))); }

constexpr double pow_const(double base, int exponent) {
  return exponent <  0 ? 1.0 / pow_const(base, -exponent) :
         exponent == 0 ? 1.                        :
         exponent == 1 ? base                      :
         base * pow_const(base, exponent - 1);
}

// atan formulae from http://mathonweb.com/algebra_e-book.htm
// x - x^3/3 + x^5/5 - x^7/7+x^9/9  etc.
constexpr double atan_poly_helper(const double res,  const double num1,
                                  const double den1, const double delta) {
  return res < tol ? res :
         res + atan_poly_helper((num1 * delta) / (den1 + 2.) - num1 / den1,
                                num1 * delta * delta, den1 + 4., delta);
}
constexpr double atan_poly_const(const double x) {
  return x + atan_poly_helper(pow_const(x, 5) / 5. - pow_const(x, 3) / 3., pow_const(x, 7), 7., x * x);
}

// Define an M_PI_6? Define a root 3?
constexpr double atan_identity_const(const double x) {
  return x <= (2. - sqrt_const(3.)) ? atan_poly_const(x) :
         (M_PI_2 / 3.) + atan_poly_const((sqrt_const(3.) * x - 1) / (sqrt_const(3.) + x));
}

constexpr double atan_cmplmntry_const(const double x) {
  return (x < 1) ? atan_identity_const(x) : M_PI_2 - atan_identity_const(1 / x);
}

constexpr double atan_const(const double x) {
  return (x >= 0) ? atan_cmplmntry_const(x) : -atan_cmplmntry_const(-x);
}

constexpr double atan2_const(const double y, const double x) {
  return           x >  0 ? atan_const(y / x)        :
                   y >= 0 && x <  0 ? atan_const(y / x) + M_PI :
                   y <  0 && x <  0 ? atan_const(y / x) - M_PI :
                   y >  0 && x == 0 ?  M_PI_2          :
                   y <  0 && x == 0 ? -M_PI_2          : 0;   // 0 == undefined
}

constexpr double nearest_const(double x) { return (x - 0.5) > (int)x ? (int)(x + 0.5) : (int)x; }

constexpr double fraction_const(double x) {
  return (x - 0.5) > (int)x ? -(((double)(int)(x + 0.5)) - x) : x - ((double)(int)(x));
}

constexpr
double exp_helper(const double r) {
  return 1.0 + r + pow_const(r, 2) / 2.0   + pow_const(r, 3) / 6.0   +
         pow_const(r, 4) / 24.0  + pow_const(r, 5) / 120.0 +
         pow_const(r, 6) / 720.0 + pow_const(r, 7) / 5040.0;
}

// exp_const(x) = e^n . e^r (where n is an integer, and -0.5 > r < 0.5
// exp_const(r) = e^r = 1 + r + r^2/2 + r^3/6 + r^4/24 + r^5/120
constexpr double exp_const(const double x) { return pow_const(M_E, nearest_const(x)) * exp_helper(fraction_const(x)); }

constexpr double mantissa_const(const double x) {
  return x >= 10.0 ? mantissa_const(x *  0.1) :
         x <  1.0  ? mantissa_const(x * 10.0) :
         x;
}

// log_const(m) = log_const(sqrt_const(m)^2) = 2 x log_const( sqrt_const(m) )
// log_const(x) = log_const(m x 10^p) = 0.86858896 ln( sqrt_const(m) ) + p
constexpr int exponent_helper(const double x, const int e) {
  return x >= 10.0 ? exponent_helper(x *  0.1, e + 1) :
         x <  1.0  ? exponent_helper(x * 10.0, e - 1) :
         e;
}

constexpr int exponent_const(const double x) { return exponent_helper(x, 0); }

constexpr double log_helper2(const double y) {
  return 2.0 * (y + pow_const(y, 3) / 3.0 + pow_const(y, 5) / 5.0 +
                pow_const(y, 7) / 7.0 + pow_const(y, 9) / 9.0 + pow_const(y, 11) / 11.0);
}

// log in the range 1-sqrt_const(10)
constexpr double log_helper(const double x) { return log_helper2((x - 1.0) / (x + 1.0)); }

// n.b. log 10 is 2.3025851
// n.b. log m = log (sqrt_const(m)^2) = 2 * log sqrt_const(m)
constexpr double log_const(const double x) {
  return x == 0 ? -std::numeric_limits<double>::infinity() :
         x <  0 ?  std::numeric_limits<double>::quiet_NaN() :
         2.0 * log_helper(sqrt_const(mantissa_const(x))) + 2.3025851 * exponent_const(x);
}
