/**
 * @file
 *
 * Containes pose data structure
 */

#pragma once

#include <cstddef>

#include "thirdparty/simplevectors.hpp"

namespace hermite {
using svector::Vector;

/**
 * @brief Pose
 *
 * Poses in multiple dimensions can use multiple Pose objects.
 *
 * It is recommended to create a pose using makePose().
 */
template <std::size_t D> struct Pose {
  Vector<D> pos; //!< Position value
  Vector<D> vel; //!< Velocity value
  Vector<D> acc; //!< Acceleration value
  double time;   //!< Time from beginning to reach this pose
};

/**
 * @brief Makes pose from time and position information
 *
 * @note Makes velocity and acceleration zero.
 *
 * @param time Time of pose from the beginning of path planning
 * @param pos Position value
 *
 * @returns Pose object.
 */
template <std::size_t D>
inline Pose<D> makePose(const double time, const Vector<D> pos) {
  Pose<D> res;
  res.time = time;
  res.pos = pos;
  return res;
}

/**
 * @brief Makes pose from time, position, and velocity information
 *
 * @note Makes acceleration zero.
 *
 * @param time Time of pose from the beginning of path planning
 * @param pos Position value
 * @param vel Velocity value
 *
 * @returns Pose object.
 */
template <std::size_t D>
inline Pose<D> makePose(const double time, const Vector<D> pos,
                        const Vector<D> vel) {
  Pose<D> res;
  res.time = time;
  res.pos = pos;
  res.vel = vel;
  return res;
}

/**
 * @brief Makes pose from time, position, velocity, and acceleration information
 *
 * @param time Time of pose from the beginning of path planning
 * @param pos Position value
 * @param vel Velocity value
 * @param acc Acceleration value
 *
 * @returns Pose object.
 */
template <std::size_t D>
inline Pose<D> makePose(const double time, const Vector<D> pos,
                        const Vector<D> vel, const Vector<D> acc) {
  Pose<D> res;
  res.time = time;
  res.pos = pos;
  res.vel = vel;
  res.acc = acc;
  return res;
}
} // namespace hermite
