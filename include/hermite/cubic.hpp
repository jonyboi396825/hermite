/**
 * @file
 *
 * Main cubic spline class
 */

#include <cstddef>
#include <vector>

#include "hermite/base_interpol.hpp"
#include "hermite/pose.hpp"
#include "hermite/spline/spline_vec.hpp"

namespace hermite {
/**
 * @brief A natural cubic spline.
 *
 * Given a set of poses, this class interpolates a path. However, only the
 * velocity for the first and last poses need to be known. This class will
 * determine velocities for the other positions such that the second derivatives
 * match.
 *
 * This class takes in an std::vector of poses and interpolates from that
 * vector. This vector should be obtained using the Hermite class's
 * getAllWaypoints() method. Although the vector can be created yourself, you
 * need to be careful and make sure there are no repeated
 * points in time, or it may lead to undefined behavior.
 *
 * The advantage of using this over Hermite is that you obtain C2 continuity.
 * However, the path must be regenerated after changing one position, and it
 * will affect the entire curve, giving up local control. Additionally, you
 * cannot specify the velocities of the waypoints in between the start and end
 * point.
 *
 * Note that this class can be only used for one unique set of
 * points. To use it for another set, it is highly recommended to insert/delete
 * points from the Hermite class and use the output of
 * Hermite::getAllWaypoints() to generate the points in the constructor to
 * ensure defined behavior.
 */
template <std::size_t D> class Cubic : public BaseInterpol<D> {
public:
  /**
   * @brief Default constructor
   *
   * Initializes with zero waypoints
   */
  Cubic() = default;

  /**
   * @brief Constructor
   *
   * It is highly recommended to insert the points in the Hermite class first,
   * and then use the output of Hermite::getAllWaypoints() for the constructor.
   *
   * @param waypoints A list of waypoints
   *
   * @note Sorts the waypoints
   * @note Make sure that there are no two waypoints that share the same time,
   * or there will be undefined behavior.
   */
  Cubic(const std::vector<Pose<D>> &waypoints) : m_waypoints{waypoints} {
    std::sort(m_waypoints.begin(), m_waypoints.end(),
              [](const Pose<D> &a, const Pose<D> &b) {
                return a.getPos() < b.getPos();
              });
    m_spl = SplineVec<D>{m_waypoints};
  }

  /**
   * @brief Copy constructor
   */
  Cubic(const Cubic<D> &other)
      : m_waypoints{other.m_waypoints}, m_spl{other.m_spl} {}

  /**
   * @brief Assignment operator
   */
  Cubic<D> &operator=(const Cubic<D> &other) {
    m_waypoints = other.m_waypoints;
    m_spl = other.m_spl;
  }

  /**
   * @brief Destructor
   */
  ~Cubic() override = default;

  /**
   * @brief Gets a list of all waypoints
   *
   * @returns A list of all waypoints, sorted in order of time.
   */
  std::vector<Pose<D>> getAllWaypoints() const { return m_waypoints; }

  /**
   * @brief Gets the lower bound of the domain of the piecewise spline function,
   * which is the first time (lowest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   * @note Assumes that the list was sorted from the constructor.
   *
   * @returns The first time measurement
   */
  double getLowestTime() const {
    if (m_waypoints.size() == 0) {
      return 0;
    }

    return m_waypoints[0].getTime();
  }

  /**
   * @brief Gets the upper bound of the domain of the piecewise spline function,
   * which is the first time (lowest t-value) listed in the waypoints.
   *
   * @note If there are no waypoints, then returns 0
   * @note Assumes that the list was sorted from the constructor.
   *
   * @returns The last time measurement
   */
  double getHighestTime() const {
    if (m_waypoints.size() == 0) {
      return 0;
    }

    return m_waypoints[m_waypoints.size() - 1].getTime();
  }

  /**
   * @brief Gets position at a certain time
   *
   * Same as calling operator()()
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   *
   * @param t Time
   *
   * @returns Position
   */
  Vector<D> getPos(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    return m_spl.splpos(t);
  }

  /**
   * @brief Gets velocity at a certain time
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   *
   * @param t Time
   *
   * @returns Velocity
   */
  Vector<D> getVel(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    return m_spl.splvel(t);
  }

  /**
   * @brief Gets acceleration of the function at a certain time
   *
   * @note If time is outside the domain of time from the given points, then it
   * calculates the value for the function whose domain is nearest to t.
   * @note If number of waypoints is less than or equal to 1, then returns a
   * zero vector.
   *
   * @param t Time
   *
   * @returns Acceleration
   */
  Vector<D> getAcc(const double t) const override {
    Vector<D> res;
    if (m_waypoints.size() < 2) {
      return res;
    }

    return m_spl.splacc(t);
  }

  double getMaxDistance(const double timeStep) const;
  double getMaxSpeed(const double timeStep) const;
  double getMaxAcceleration(const double timeStep) const;

  double getLength(const double timeStep) const;

private:
  std::vector<Pose<D>> m_waypoints;
  SplineVec<D> m_spl;
};
} // namespace hermite
