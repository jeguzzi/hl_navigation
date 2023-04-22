/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_ORCA_H_
#define HL_NAVIGATION_BEHAVIOR_ORCA_H_

#include <memory>

#include "hl_navigation/behavior.h"
#include "hl_navigation/property.h"
#include "hl_navigation/states/geometric.h"
#include "hl_navigation_export.h"

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
 *
 * *Properties*: time_horizon (int), effective_center (bool)
 * 
 * *State*: \ref GeometricState 
 */
class HL_NAVIGATION_EXPORT ORCABehavior : public Behavior {
 public:
  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  ORCABehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
               float radius = 0.0f);
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
   * @brief      Determines if an effective center is being used.
   *
   * Using an effective center placed with an offset towards the front, allows
   * to consider the kinematics as holonomic instead of a two-wheeled
   * differential drive. See
   *
   *     J. Snape, J. van den Berg, S. J. Guy, and D. Manocha,
   *     "Smooth and collision-free navigation for multiple robots under
   *     differential-drive constraints," in 2010 IEEE/RSJ International
   *     Conference on Intelligent, Robots and Systems, 2010, pp. 4584–4589.
   *
   * with ``D=L/2``.
   *
   * Only possibly true if the  kinematics is wheeled and constrained.
   *
   * @return     True if using effective center, False otherwise.
   */
  bool is_using_effective_center() const {
    if (!kinematics) return false;
    return use_effective_center && kinematics->is_wheeled() &&
           kinematics->dof() == 2;
  }
  /**
   * @brief      Specifies if the kinematics should be using an shifted
   * effective center, see \ref set_time_horizon
   *
   * @param[in]  value Whenever is should use an effective center when
   * kinematics is wheeled and constrained.
   */
  void should_use_effective_center(bool value) { use_effective_center = value; }

  void set_time_step(float value);
  float get_time_step() const;

  /**
   * @private
   */
  virtual const Properties& get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties =
      Properties{
          {"time_horizon",
           make_property<float, ORCABehavior>(&ORCABehavior::get_time_horizon,
                                              &ORCABehavior::set_time_horizon,
                                              10.0f, "Time horizon")},
          {"effective_center",
           make_property<bool, ORCABehavior>(
               &ORCABehavior::is_using_effective_center,
               &ORCABehavior::should_use_effective_center, false,
               "Whenever to use an effective center to handle non-holonomic "
               "kinematics")},
      } +
      Behavior::properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /** 
   * @private
  */
  EnvironmentState * get_environment_state() override {
    return &state;
  }

 protected:
  Twist2 twist_towards_velocity(const Vector2& absolute_velocity,
                                bool relative) override;
  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override;

 private:
  GeometricState state;
  bool use_effective_center;
  float D;
  std::unique_ptr<RVO::Agent> _RVOAgent;
  std::vector<std::unique_ptr<const RVO::Agent>> rvo_neighbors;
  std::vector<std::unique_ptr<const RVO::Obstacle>> rvo_obstacles;

  void add_line_obstacle(const LineSegment& line);
  void add_neighbor(const Neighbor& disc, float range, bool push_away = false,
                    float epsilon = 2e-3);
  void add_obstacle(const Disc& disc, float range, bool push_away = false,
                    float epsilon = 2e-3);
  void prepare_line_obstacles();
  void prepare();

 private:
  inline static std::string type = register_type<ORCABehavior>("ORCA");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_ORCA_H_
