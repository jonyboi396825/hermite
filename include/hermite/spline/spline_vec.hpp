/**
 * @file
 *
 * Low-level spline operations with vectors and STL containers
 */

#pragma once

#include <cstddef>
#include <vector>

#include "hermite/pose.hpp"
#include "hermite/spline/spline_impl.hpp"
#include "hermite/thirdparty/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * Generates second derivatives of splines
 *
 * @param poses The list of poses. Velocities of poses 1...n-2 will be ignored.
 *
 * @note Assumes that the size of poses is greater than or equal to 2, it is
 * sorted by time, and there are no repeated times. Otherwise, there will be
 * undefined behavior.
 *
 * @returns The list of accelerations
 */
template <std::size_t D>
inline std::vector<Vector<D>> spline(const std::vector<Pose<D>> &poses) {
  const std::size_t n = poses.size();
  std::vector<Vector<D>> res(n);

  // solve each dimension independently
  for (std::size_t dim = 0; dim < D; dim++) {
    std::vector<double> t(n);
    std::vector<double> y(n);

    for (std::size_t i = 0; i < n; i++) {
      t[i] = poses[i].getTime();
      y[i] = poses[i].getPos()[dim];
    }

    const double yp1 = poses[0].getVel()[dim];
    const double ypn = poses[n - 1].getVel()[dim];

    std::vector<double> ydd(n);
    spline(t.data(), y.data(), static_cast<int>(n), yp1, ypn, ydd.data());

    for (std::size_t i = 0; i < n; i++) {
      res[i][dim] = ydd[i];
    }
  }

  return res;
}

/**
 * Gets position value given a time input
 *
 * @param poses A list of poses
 * @param accs The list of accelerations from calling spline()
 * @param t Time input
 *
 * @note Assumes that the size of poses is greater than or equal to 2, it is
 * sorted by time, and there are no repeated times. Otherwise, there will be
 * undefined behavior.
 * @note Assumes that the sizes of poses and accs are the same, otherwise
 * results in undefined behavior.
 * @note Because of copying, the complexity is quadratic rather than
 * logarithmic, so it can be slow for a large number of data points.
 *
 * @returns Position vector at a given time
 */
template <std::size_t D>
inline Vector<D> splpos(const std::vector<Pose<D>> &poses,
                        const std::vector<Vector<D>> &accs, const double t) {
  const std::size_t n = poses.size();
  Vector<D> res;

  // solve each dimension independently
  for (std::size_t dim = 0; dim < D; dim++) {
    std::vector<double> ta(n);
    std::vector<double> y(n);
    std::vector<double> accsd(n);

    for (std::size_t i = 0; i < n; i++) {
      ta[i] = poses[i].getTime();
      y[i] = poses[i].getPos()[dim];
      accsd[i] = accs[i][dim];
    }

    double output;
    splpos(ta.data(), y.data(), accsd.data(), static_cast<int>(n), t, &output);
    res[dim] = output;
  }

  return res;
}

/**
 * Gets velocity value given a time input
 *
 * @param poses A list of poses
 * @param accs The list of accelerations from calling spline()
 * @param t Time input
 *
 * @note Assumes that the size of poses is greater than or equal to 2, it is
 * sorted by time, and there are no repeated times. Otherwise, there will be
 * undefined behavior.
 * @note Assumes that the sizes of poses and accs are the same, otherwise
 * results in undefined behavior.
 * @note Because of copying, the complexity is quadratic rather than
 * logarithmic, so it can be slow for a large number of data points.
 *
 * @returns Velocity vector at a given time
 */
template <std::size_t D>
inline Vector<D> splvel(const std::vector<Pose<D>> &poses,
                        const std::vector<Vector<D>> &accs, const double t) {
  const std::size_t n = poses.size();
  Vector<D> res;

  // solve each dimension independently
  for (std::size_t dim = 0; dim < D; dim++) {
    std::vector<double> ta(n);
    std::vector<double> y(n);
    std::vector<double> accsd(n);

    for (std::size_t i = 0; i < n; i++) {
      ta[i] = poses[i].getTime();
      y[i] = poses[i].getPos()[dim];
      accsd[i] = accs[i][dim];
    }

    double output;
    splvel(ta.data(), y.data(), accsd.data(), static_cast<int>(n), t, &output);
    res[dim] = output;
  }

  return res;
}

/**
 * Gets acceleration value given a time input
 *
 * @param poses A list of poses
 * @param accs The list of accelerations from calling spline()
 * @param t Time input
 *
 * @note Assumes that the size of poses is greater than or equal to 2, it is
 * sorted by time, and there are no repeated times. Otherwise, there will be
 * undefined behavior.
 * @note Assumes that the sizes of poses and accs are the same, otherwise
 * results in undefined behavior.
 * @note Because of copying, the complexity is quadratic rather than
 * logarithmic, so it can be slow for a large number of data points.
 *
 * @returns Acceleration vector at a given time
 */
template <std::size_t D>
inline Vector<D> splacc(const std::vector<Pose<D>> &poses,
                        const std::vector<Vector<D>> &accs, const double t) {
  const std::size_t n = poses.size();
  Vector<D> res;

  // solve each dimension independently
  for (std::size_t dim = 0; dim < D; dim++) {
    std::vector<double> ta(n);
    std::vector<double> y(n);
    std::vector<double> accsd(n);

    for (std::size_t i = 0; i < n; i++) {
      ta[i] = poses[i].getTime();
      y[i] = poses[i].getPos()[dim];
      accsd[i] = accs[i][dim];
    }

    double output;
    splacc(ta.data(), y.data(), accsd.data(), static_cast<int>(n), t, &output);
    res[dim] = output;
  }

  return res;
}
} // namespace hermite
