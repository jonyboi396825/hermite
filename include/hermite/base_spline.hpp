/**
 * @file
 *
 * A base class for interpolating splines
 */

#include <cstddef>

#include "hermite/base_interpol.hpp"
#include "hermite/thirdparty/simplevectors.hpp"

namespace hermite {
/**
 * @file
 *
 * Abstract base class for interpolating splines
 */
template <std::size_t D> class BaseSpline : public BaseInterpol<D> {
public:
  /**
   * @brief Destructor
   */
  virtual ~BaseSpline() = default;

  /**
   * @brief Gets the lower bound of the domain of the piecewise spline function,
   * which is the first time (lowest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The first time measurement
   */
  virtual double getLowestTime() const = 0;

  /**
   * @brief Gets the upper bound of the domain of the piecewise spline function,
   * which is the last time (highest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   *
   * @returns The last time measurement
   */
  virtual double getHighestTime() const = 0;

  /**
   * @brief Gets maximum distance from origin
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no waypoints, returns 0.
   *
   * @returns Maximum distance from the origin.
   */
  virtual double getMaxDistance(const double timeStep) const = 0;

  /**
   * @brief Gets maximum speed
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no poses, returns 0.
   *
   * @returns Maximum speed.
   */
  virtual double getMaxSpeed(const double timeStep) const = 0;

  /**
   * @brief Gets maximum magnitude of acceleration
   *
   * @param timeStep The time step to try for the absolute maximum
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If no poses, returns 0.
   *
   * @returns Magnitude of maximum acceleration.
   */
  virtual double getMaxAcceleration(const double timeStep) const = 0;
  /**
   * @brief Gets arc length
   *
   * @param timeStep The time step to try for the arc length
   *
   * @note This function will take much longer for smaller timesteps.
   * Recommended is between 0.001 and 0.1, but this also depends on the domain
   * of your function.
   * @note If zero or one poses, returns 0.
   *
   * @returns Arc length
   */
  virtual double getLength(const double timeStep) const = 0;
};
} // namespace hermite
