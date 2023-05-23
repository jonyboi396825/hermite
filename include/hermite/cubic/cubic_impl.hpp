/**
 * @file
 *
 * A lower-level spline solver for a natural cubic spline
 */

#pragma once

#include <vector>

namespace hermite {
/**
 * @brief Calculates second derivatives at tabulated points
 *
 * https://archive.org/details/NumericalRecipes/page/n139/mode/2up
 *
 * @param t The time values of each point where splines are supposed to join
 * up.
 * @param y The y-values of each know point.
 * @param n Number of points.
 * @param yp1 The velocity at the first point
 * @param ypn The velocity at the last point
 * @param y2 Output array, filled with second derivatives at each point.
 */
inline void spline(const double t[], const double y[], const int n,
                   const double yp1, const double ypn, double y2[]) {
  int i, k;
  double p, qn, sig, un;
  std::vector<double> u;
  u.resize(n + 2);

  if (yp1 > 0.99e30) {
    // y2[1] = u[1] = 0.0;
    y2[0] = u[0] = 0.0;
  } else {
    y2[0] = -0.5;
    u[0] = (3.0 / (t[1] - t[0])) * ((y[1] - y[0]) / (t[1] - t[0]) - yp1);
  }

  for (i = 1; i <= n - 2; i++) {
    sig = (t[i] - t[i - 1]) / (t[i + 1] - t[i - 1]);
    p = sig * y2[i - 1] + 2.0;
    y2[i] = (sig - 1.0) / p;
    u[i] = (y[i + 1] - y[i]) / (t[i + 1] - t[i]) -
           (y[i] - y[i - 1]) / (t[i] - t[i - 1]);
    u[i] = (6.0 * u[i] / (t[i + 1] - t[i - 1]) - sig * u[i - 1]) / p;
  }

  if (ypn > 0.99e30) {
    qn = un = 0.0;
  } else {
    qn = 0.5;
    un = (3.0 / (t[n - 1] - t[n - 2])) *
         (ypn - (y[n - 1] - y[n - 2]) / (t[n - 1] - t[n - 2]));
  }
  y2[n - 1] = (un - qn * u[n - 2]) / (qn * y2[n - 2] + 1.0);

  for (k = n - 2; k >= 0; k--) {
    y2[k] = y2[k] * y2[k + 1] + u[k];
  }
}

/**
 * @brief Calculates position of spline
 *
 * https://archive.org/details/NumericalRecipes/page/n139/mode/2up
 *
 * @param xa The time values of each point where splines are supposed to join
 * up.
 * @param ya The y-values of each know point.
 * @param y2a Second derivatives, calculated from spline()
 * @param n Number of points
 * @param x Point to evaluate
 * @param y Pointer to result
 *
 * @returns True on success, false on failure
 */
inline bool splpos(const double xa[], const double ya[], const double y2a[],
                   const int n, const double x, double *y) {
  int klo, khi, k;
  double h, b, a;

  klo = 0;
  khi = n - 1;
  while (khi - klo > 1) {
    k = (khi + klo) >> 1;
    if (xa[k] > x) {
      khi = k;
    } else {
      klo = k;
    }
  }

  h = xa[khi] - xa[klo];
  if (h == 0.0) {
    return false;
  }

  a = (xa[khi] - x) / h;
  b = (x - xa[klo]) / h;
  *y =
      a * ya[klo] + b * ya[khi] +
      ((a * a * a - a) * y2a[klo] + (b * b * b - b) * y2a[khi]) * (h * h) / 6.0;
  return true;
}

/**
 * @brief Calculates velocity of spline
 *
 * https://archive.org/details/NumericalRecipes/page/n139/mode/2up
 *
 * @param xa The time values of each point where splines are supposed to join
 * up.
 * @param ya The y-values of each know point.
 * @param y2a Second derivatives, calculated from spline()
 * @param n Number of points
 * @param x Point to evaluate
 * @param y Pointer to result
 *
 * @returns True on success, false on failure
 */
inline bool splvel(const double xa[], const double ya[], const double y2a[],
                   const int n, const double x, double *y) {
  int klo, khi, k;
  double h, b, a;

  klo = 0;
  khi = n - 1;
  while (khi - klo > 1) {
    k = (khi + klo) >> 1;
    if (xa[k] > x) {
      khi = k;
    } else {
      klo = k;
    }
  }

  h = xa[khi] - xa[klo];
  if (h == 0.0) {
    return false;
  }

  a = (xa[khi] - x) / h;
  b = (x - xa[klo]) / h;
  *y = (-1 / h) * ya[klo] + (1 / h) * ya[khi] +
       ((3 * a * a - 1) * y2a[klo] * (-1 / h) +
        (3 * b * b - 1) * y2a[khi] * (1 / h)) *
           (h * h) / 6.0;
  return true;
}

/**
 * @brief Calculates acceleration of spline
 *
 * https://archive.org/details/NumericalRecipes/page/n139/mode/2up
 *
 * @param xa The time values of each point where splines are supposed to join
 * up.
 * @param ya The y-values of each know point.
 * @param y2a Second derivatives, calculated from spline()
 * @param n Number of points
 * @param x Point to evaluate
 * @param y Pointer to result
 *
 * @returns True on success, false on failure
 */
inline bool splacc(const double xa[], const double ya[], const double y2a[],
                   const int n, const double x, double *y) {
  int klo, khi, k;
  double h, b, a;

  klo = 0;
  khi = n - 1;
  while (khi - klo > 1) {
    k = (khi + klo) >> 1;
    if (xa[k] > x) {
      khi = k;
    } else {
      klo = k;
    }
  }

  h = xa[khi] - xa[klo];
  if (h == 0.0) {
    return false;
  }

  (void)ya;

  a = (xa[khi] - x) / h;
  b = (x - xa[klo]) / h;
  *y = y2a[klo] * a + y2a[khi] * b;
  return true;
}
} // namespace hermite
