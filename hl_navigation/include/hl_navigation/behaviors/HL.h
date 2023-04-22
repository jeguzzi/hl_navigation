/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_HL_H_
#define HL_NAVIGATION_BEHAVIOR_HL_H_

#include <algorithm>
#include <memory>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/collision_computation.h"
#include "hl_navigation/property.h"
#include "hl_navigation/states/geometric.h"
#include "hl_navigation_export.h"

// TODO(J): verify if behavior for tau < step is correct (non smooth)

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
 *     
 * *Properties*: tau (float), eta (float), aperture (float), resolution(int)  
 */
class HL_NAVIGATION_EXPORT HLBehavior : public Behavior, public GeometricState {
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

  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  HLBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
             float radius = 0.0f)
      : Behavior(kinematics, radius),
        GeometricState(),
        effective_horizon(0.0f),
        tau(default_tau),
        eta(default_eta),
        aperture(default_aperture),
        resolution(std::min(default_resolution, max_resolution)),
        collision_computation() {}
  ~HLBehavior() = default;

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
    resolution = std::clamp<unsigned>(value, 1, max_resolution);
  }
  /**
   * @brief      Convenience method that return the size of an angular segment
   * in \f$[-\alpha, \alpha]\f$ to search for optimal directions.
   *
   * @return     The angular resolution for the optimal direction search in
   * radians.
   */
  Radians get_angular_resolution() const { return 2 * aperture / resolution; }

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


  /** @private
  */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * Properties: tau, eta, aperture, and resolution
   * @private
   */
  static inline std::map<std::string, Property> properties =
      Properties{
          {"tau",
           make_property<float, HLBehavior>(
               &HLBehavior::get_tau, &HLBehavior::set_tau, default_tau, "Tau")},
          {"eta",
           make_property<float, HLBehavior>(
               &HLBehavior::get_eta, &HLBehavior::set_eta, default_eta, "Eta")},
          {"aperture", make_property<float, HLBehavior>(
                           &HLBehavior::get_aperture, &HLBehavior::set_aperture,
                           default_aperture, "Aperture angle")},
          {"resolution",
           make_property<int, HLBehavior>(&HLBehavior::get_resolution,
                                          &HLBehavior::set_resolution,
                                          default_resolution, "Safety margin")},
      } +
      Behavior::properties;

  /** @private
  */
  std::string get_type() const override { return type; }

 protected:
  float effective_horizon;
  float tau;
  float eta;
  Radians aperture;
  unsigned int resolution;
  CollisionComputation collision_computation;

  /**
   * @brief      Override \ref Behavior::cmd_twist adding target velocity
   * relaxation
   *
   * The target velocities (twist or wheel speeds, depending on the \ref
   * get_kinematics) are relaxed over time \f$\eta\f$ as \f$ \dot v = (v_t - v) /
   * \eta \f$, where \f$v_t\f$ is the instantaneous desired value computed by
   * \ref Behavior::cmd_twist.
   *
   * If \f$\eta=0\f$, no relaxation is performed and the desired target velocity
   * is returned.
   *
   */
  Twist2 cmd_twist(float time_step, Mode mode, bool relative,
                   bool set_as_actuated) override;

  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override;

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

  DiscCache make_neighbor_cache(const Neighbor &neighbor,
                                bool push_away = false, float epsilon = 2e-3);
  DiscCache make_obstacle_cache(const Disc &obstacle, bool push_away = false,
                                float epsilon = 2e-3);


 private:
  inline static std::string type = register_type<HLBehavior>("HL");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_HL_H_
