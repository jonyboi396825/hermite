/**
 * @file
 *
 * Interpolation on the unit interval
 */

#pragma once

#include <cstddef>

#include "base.hpp"
#include "constants.hpp"
#include "thirdparty/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Interpolates on the unit interval
 *
 * Calculates one Hermite spline section on the unit interval [0, 1] given a
 * starting point and velocity at time t=0 and an ending point and velocity at
 * t=tf.
 *
 * The template represents the number of dimensions to calculate in. For
 * example, for 2 dimensions, the position and velocity functions will output a
 * 2D vector.
 */
template <std::size_t D> class HermiteUnit : public BaseInterpol<D> {
public:
  HermiteUnit() = delete;

  /**
   * @brief Constructor
   *
   * @param p0 Initial position vector
   * @param p1 Initial position vector
   * @param v0 Initial position vector
   * @param v1 Initial position vector
   */
  HermiteUnit(const Vector<D> p0, const Vector<D> p1, const Vector<D> v0,
              const Vector<D> v1)
      : m_p0{p0}, m_p1{p1}, m_v0{v0}, m_v1{v1} {}

  /**
   * @brief Destructor
   */
  ~HermiteUnit() = default;

private:
  Vector<D> m_p0;
  Vector<D> m_p1;
  Vector<D> m_v0;
  Vector<D> m_v1;
};
} // namespace hermite
