/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_HL_H_
#define HL_NAVIGATION_BEHAVIOR_HL_H_

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/collision_computation.h"
#include "hl_navigation/state/geometric.h"

// TODO(J): verify if behavior for tau < step is correct (non smooth)

// trajectory planning

namespace hl_navigation {

/**
 * @brief      Human-like obstacle avoidance behavior.
 *
 * The behavior inspired by how pedestrian move, has been originally presented
 * in
 *
 *     Guzzi, J.; Giusti, A.; Gambardella, L.M.; Theraulaz, G.; Di Caro, G.A.,
 *     "Human-friendly robot navigation in dynamic environments,"
 *     Robotics and Automation (ICRA), 2013 IEEE International Conference on,
 *     vol., no., pp.423,430, 6-10 May 2013
 */
class HLBehavior : public Behavior, public GeometricState {
 public:
  /**
   * Default \f$\eta\f$
   */
  static constexpr float default_eta = 0.5f;
  /**
   * Default \f$\tau\f$
   */
  static constexpr float default_tau = 0.125f;
  /**
   * Default aperture (full circular sector)
   */
  static constexpr float default_aperture = M_PI;
  /**
   * Maximal resolution
   */
  static constexpr unsigned max_resolution = 361;
  /**
   * Default resolution. Should be less than \ref max_resolution
   */
  static constexpr unsigned default_resolution = 101;

  HLBehavior(std::shared_ptr<Kinematic> kinematic, float radius)
      : Behavior(kinematic, radius),
        GeometricState(),
        effective_horizon(0.0f),
        tau(default_tau),
        eta(default_eta),
        aperture(default_aperture),
        resolution(std::min(default_resolution, max_resolution)),
        collision_computation() {}
  ~HLBehavior();

  // -------------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the relaxation time \f$\eta\f$.
   *
   * @return     \f$\eta\f$
   */
  float get_eta() const { return eta; }
  /**
   * @brief      Sets the relaxation time \f$\eta\f$.
   *
   * @param[in]  value  A positive value. If zero, relaxation is disabled.
   */
  void set_eta(float value) { eta = value; }
  /**
   * @brief      Gets the  time \f$\tau\f$ that the behavior keep away from
   * collisions Higher values lead to slower speeds.
   *
   * @return     \f$\tau\f$
   */
  float get_tau() const { return tau; }
  /**
   * @brief      Sets the time \f$\tau\f$ that the behavior keep away from
   * collisions. Higher values lead to slower speeds.
   *
   * @param[in]  value  A strict positive value.
   */
  void set_tau(float value) { tau = value; }
  /**
   * @brief      Gets the aperture \f$\alpha\f$: desired velocity is searched on
   * a circular sector \f$[-\alpha, \alpha]\f$.
   *
   * @return     The positive \f$\alpha\f$ in radians.
   */
  Radians get_aperture() const { return aperture; }
  /**
   * @brief      Sets the aperture, see \ref get_aperture
   *
   * @param[in]  value  A positive value
   */
  void set_aperture(Radians value) { aperture = value; }
  /**
   * @brief      Gets the number of subdivision of \f$[-\alpha, \alpha]\f$ to
   * search for optimal directions.
   *
   * @return     The resolution.
   */
  unsigned get_resolution() const { return resolution; }
  /**
   * @brief      Sets the number of subdivision of \f$[-\alpha, \alpha]\f$ to
   * search for optimal directions.
   *
   * @param[in]  value  A strict positive value. The larger the value, the more
   * precise the motion but also the more expensive the computations.
   */
  void set_resolution(unsigned value) {
    resolution = std::min<unsigned>(value, max_resolution);
  }
  /**
   * @brief      Convenience method that return the size of an angular segment
   * in \f$[-\alpha, \alpha]\f$ to search for optimal directions.
   *
   * @return     The angular resolution for the optimal direction search in
   * radians.
   */
  Radians get_angular_resolution() const { return 2 * aperture / resolution; }

  // -------------------------- CONTROL

  /**
   * @brief      Override \ref Behavior::cmd_twist adding target velocity
   * relaxation
   *
   * The target velocities (twist or wheel speeds, depending on the \ref
   * kinematic) are relaxed over time \f$\eta\f$ as \f$ \dot v = (v_t - v) /
   * \eta \f$, where \f$v_t\f$ is the instantaneous desired value computed by
   * Behavior::cmd_twist.
   *
   * If \f$\eta=0\f$, no relaxation is performed and the desired target velocity
   * is returned.
   *
   * @param[in]  time_step        The control time step, used to integrate
   * velocity relaxation.
   * @param[in]  relative         The desired frame of reference for the twist:
   *                              set it to true for the agent's own frame and
   * to false for the world fixed frame.
   * @param[in]  mode             Should the agent move towards the \ref
   * Behavior::get_target_position (\ref Behavior::Mode::move), turn-in-place
   * towards \ref Behavior::get_target_orientation, or stop moving.
   * @param[in]  set_as_actuated  If set, it assumes that the control command
   * will be actuated, therefore setting Behavior::actuated_twist to the
   * returned value. If not set, the user should call
   * Behavior::set_actuated_twist before querying for a new control commands. it
   * uses the old actuated control command as \f$v\f$ in the velocity
   * relaxation.
   *
   * @return     The control command as a twist in the specified frame.
   */
  Twist2 cmd_twist(float time_step, bool relative, Mode mode = Mode::move,
                   bool set_as_actuated = true) override;

  /**
   * @brief      Gets the free distance to collision in \f$[-\alpha, \alpha]\f$
   * at regular intervals.
   *
   * @param[in]  assuming_static  If True, all obstacles are assumed static.
   *
   * @return     A vector of pairs <angle, distance> of size \ref
   * get_resolution. Angles are in the agent frame.
   */
  CollisionComputation::CollisionMap get_collision_distance(
      bool assuming_static = false);

 protected:
  float effective_horizon;
  float tau;
  float eta;
  Radians aperture;
  unsigned int resolution;
  CollisionComputation collision_computation;

  Vector2 compute_desired_velocity() override;

  void prepare();
  Vector2 compute_repulsive_force(bool &inside_obstacle);
  Twist2 relax(const Twist2 &twist, float dt) const;
  unsigned int index_of_relative_angle(Radians relative_angle);
  float distance_to_segment(const LineSegment &line, Radians absolute_angle);
  float dist_for_angle(const DiscCache *agent, Radians angle);
  float compute_distance_to_collision_at_relative_angle(Radians relative_angle,
                                                        float *staticCache);
  float feared_distance_to_collision_at_relative_angle(Radians angle);
  float static_dist_for_angle(const DiscCache *agent, Radians angle);
  float distance_to_collision_at_relative_angle(Radians angle);

  DiscCache make_neighbor_cache(const Disc &neighbor, bool push_away = false,
                                float epsilon = 2e-3);
  DiscCache make_obstacle_cache(const Disc &obstacle, bool push_away = false,
                                float epsilon = 2e-3);

 private:
  static const char *name;
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_HL_H_
