/**
 * @file
 *
 * Low-level spline operations with vectors and STL containers
 */

#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "hermite/pose.hpp"
#include "hermite/spline/spline_impl.hpp"
#include "hermite/thirdparty/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Spline vector calculator
 *
 * Calculates spline for vectors in C++ STL containers
 */
template <std::size_t D> class SplineVec {
public:
  /**
   * @brief Default constructor
   *
   * @note Initializing with a default constructor and then using the methods
   * will result in undefined behavior. Make sure that you are assigning the
   * current object to another one with two or more waypoints, or you check
   * bounds before using any of the methods.
   */
  SplineVec() = default;

  /**
   * @brief Constructor
   *
   * Takes a list of poses and calculates the position values and time values
   * for each pose in each dimension efficiently (linear complexity to the
   * number of waypoints, times the number of dimensions).
   *
   * @note Assumes that the size of poses is greater than or equal to 2, it is
   * sorted by time, and there are no repeated times. Otherwise, there will be
   * undefined behavior.
   *
   * @param waypoints A list of poses
   */
  SplineVec(const std::vector<Pose<D>> &waypoints) {
    const std::size_t n = waypoints.size();
    m_ts.resize(n);

    // solve each dimension independently
    for (std::size_t dim = 0; dim < D; dim++) {
      m_ys[dim].resize(n);
      m_accs[dim].resize(n);

      for (std::size_t i = 0; i < n; i++) {
        m_ts[i] = waypoints[i].getTime();
        m_ys[dim][i] = waypoints[i].getPos()[dim];
      }

      const double yp1 = waypoints[0].getVel()[dim];
      const double ypn = waypoints[n - 1].getVel()[dim];

      spline(m_ts.data(), m_ys[dim].data(), static_cast<int>(n), yp1, ypn,
             m_accs[dim].data());
    }
  }

  /**
   * @brief Copy constructor
   */
  SplineVec(const SplineVec<D> &other)
      : m_ts{other.m_ts}, m_ys{other.m_ys}, m_accs{other.m_accs} {}

  /**
   * @brief Assignment operator
   */
  SplineVec<D> &operator=(const SplineVec<D> &other) {
    // check if assigning to self
    if (this == &other) {
      return *this;
    }

    m_ts = other.m_ts;
    m_ys = other.m_ys;
    m_accs = other.m_accs;

    return *this;
  }

  /**
   * @brief Destructor
   */
  ~SplineVec() = default;

  /**
   * Gets position value given a time input
   *
   * @param t Time input
   *
   * @returns Position vector at a given time
   */
  Vector<D> splpos(const double t) {
    const std::size_t n = m_ts.size();
    Vector<D> res;

    // solve each dimension independently
    for (std::size_t dim = 0; dim < D; dim++) {
      double output;
      hermite::splpos(m_ts.data(), m_ys[dim].data(), m_accs[dim].data(),
                      static_cast<int>(n), t, &output);
      res[dim] = output;
    }

    return res;
  }

  /**
   * Gets velocity value given a time input
   *
   * @param t Time input
   *
   * @returns Velocity vector at a given time
   */
  Vector<D> splvel(const double t) {
    const std::size_t n = m_ts.size();
    Vector<D> res;

    // solve each dimension independently
    for (std::size_t dim = 0; dim < D; dim++) {
      double output;
      hermite::splvel(m_ts.data(), m_ys[dim].data(), m_accs[dim].data(),
                      static_cast<int>(n), t, &output);
      res[dim] = output;
    }

    return res;
  }

  /**
   * Gets acceleration value given a time input
   *
   * @param t Time input
   *
   * @returns Acceleration vector at a given time
   */
  Vector<D> splacc(const double t) {
    const std::size_t n = m_ts.size();
    Vector<D> res;

    // solve each dimension independently
    for (std::size_t dim = 0; dim < D; dim++) {
      double output;
      hermite::splacc(m_ts.data(), m_ys[dim].data(), m_accs[dim].data(),
                      static_cast<int>(n), t, &output);
      res[dim] = output;
    }

    return res;
  }

private:
  std::vector<double> m_ts;
  std::array<std::vector<double>, D> m_ys;
  std::array<std::vector<double>, D> m_accs;
};
} // namespace hermite
