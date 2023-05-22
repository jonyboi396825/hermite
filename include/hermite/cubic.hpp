/**
 * @file
 *
 * Main cubic spline class
 */

#include <cstddef>
#include <vector>

#include "hermite/base_interpol.hpp"
#include "hermite/hermite.hpp"
#include "hermite/pose.hpp"
#include "hermite/spline/spline_impl.hpp"

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
   * @param waypoints A list of waypoints
   *
   * @note Sorts the waypoints
   * @note Make sure that there are no two waypoints that share the same time,
   * or there will be undefined behavior.
   */
  Cubic(const std::vector<Pose<D>> &waypoints);

  Cubic(const Cubic<D> &other);

  Cubic<D> &operator=(const Cubic<D> &other);

  std::vector<Pose<D>> getAllWaypoints() const {}
  double getLowestTime() const {}
  double getHighestTime() const {}

  Vector<D> getPos();
  Vector<D> getVel();
  Vector<D> getAcc();

  double getMaxDistance();
  double getMaxSpeed();
  double getMaxAcceleration();

  double getLength();

private:
  std::vector<Pose<D>> m_waypoints;
  std::vector<double> m_ydd;
};
} // namespace hermite
