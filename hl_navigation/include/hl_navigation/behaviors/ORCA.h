/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_ORCA_H_
#define HL_NAVIGATION_BEHAVIOR_ORCA_H_

#include <memory>

#include "hl_navigation/behavior.h"
#include "hl_navigation/state/geometric.h"

namespace RVO {
class Agent;
class Obstacle;
}  // namespace RVO

namespace hl_navigation {

// TODO(J): complete the effective center -> use Holonomic.

/**
 * @brief      Optimal Reciprocal Collision Avoidance
 *
 * A wrapper of the open-source implementation from
 * http://gamma.cs.unc.edu/RVO2/
 */
class ORCABehavior : public Behavior, public GeometricState {
 public:
  ORCABehavior(std::shared_ptr<Kinematic> kinematic, float radius);
  ~ORCABehavior();

  // ---------------------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the time horizon. Collisions predicted to happen after
   * this time are ignored
   *
   * @return     The time horizon.
   */
  float get_time_horizon() const;
  /**
   * @brief      Sets the time horizon. Collisions predicted to happen after
   * this time are ignored
   *
   * @param[in]  value
   */
  void set_time_horizon(float value);
  /**
   * @brief      Determines if the kinematic uses an effective center, as
   * described
   *
   *    See J. Snape, J. van den Berg, S. J. Guy, and D. Manocha,
   *    “Smooth and collision-free navigation for multiple robots under
   * differential-drive constraints,” in 2010 IEEE/RSJ International Conference
   * on Intelligent, Robots and Systems, 2010, pp. 4584–4589. with D=L/2
   *
   * Using an effective center placed with an offset towards the front, allows
   * to consider the kinematic as holonomic instead of a two-wheeled
   * differential drive.
   *
   * @return     True if using effective center, False otherwise.
   */
  bool is_using_effective_center() const { return use_effective_center; }
  /**
   * @brief      Specifies if the kinematic should be using an shifted effective
   * center, see \ref set_time_horizon
   *
   * @param[in]  value
   */
  void should_use_effective_center(bool value) {
    if (value && kinematic->is_wheeled() && kinematic->dof() == 2) {
      use_effective_center = true;
    } else {
      use_effective_center = false;
    }
  }

  void set_time_step(float value);
  float get_time_step() const;

 protected:
  Twist2 twist_towards_velocity(const Vector2& absolute_velocity,
                                bool relative) override;
  Twist2 cmd_twist_towards_target(float dt, bool relative) override;
  Vector2 compute_desired_velocity() override;

 private:
  bool use_effective_center;
  float D;
  std::unique_ptr<RVO::Agent> _RVOAgent;
  std::vector<std::unique_ptr<const RVO::Agent>> rvo_neighbors;
  std::vector<std::unique_ptr<const RVO::Obstacle>> rvo_obstacles;
  static const char* name;

  void add_line_obstacle(const LineSegment& line);
  void add_neighbor(const Disc& d, float);
  void prepare_line_obstacles();
  void prepare();
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_ORCA_H_
