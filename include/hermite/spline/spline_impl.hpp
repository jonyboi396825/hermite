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
 * @param t The time values of each point where splines are supposed to join up.
 * @param y The y-values of each know point.
 * @param n Number of points
 * @param yp1 The velocity at the first point
 * @param ypn The velocity at the last point
 * @param y2 Output array, filled with second derivatives at each point
 */
inline void spline(double t[], double y[], int n, double yp1, double ypn,
                   double y2[]) {
  int i, k;
  double p, qn, sig, un;
  std::vector<double> u;
  u.resize(n + 1);

  if (yp1 > 0.99e30) {
    y2[1] = u[1] = 0.0;
  } else {
    y2[1] = -0.5;
    u[1] = (3.0 / (t[2] - t[1])) * ((y[2] - y[1]) / (t[2] - t[1]) - yp1);
  }

  for (i = 2; i <= n - 1; i++) {
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
    un = (3.0 / (t[n] - t[n - 1])) *
         (ypn - (y[n] - y[n - 1]) / (t[n] - t[n - 1]));
  }
  y2[n] = (un - qn * u[n - 1]) / (qn * y2[n - 1] + 1.0);

  for (k = n - 1; k >= 1; k--) {
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
inline bool splint(double xa[], double ya[], double y2a[], int n, double x,
                   double *y) {
  int klo, khi, k;
  double h, b, a;

  klo = 1;
  khi = n;
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
} // namespace hermite
