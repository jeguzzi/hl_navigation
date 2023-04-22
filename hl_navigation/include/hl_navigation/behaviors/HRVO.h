/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_HRVO_H_
#define HL_NAVIGATION_BEHAVIOR_HRVO_H_

#include <memory>

#include "hl_navigation/behavior.h"
#include "hl_navigation/states/geometric.h"
#include "hl_navigation_export.h"

namespace HRVO {
class Agent;
}

namespace hl_navigation {

// DONE(J 2023): verify DIFFERENTIAL_DRIVE ->
// no need to change anything as we take care of it directly

/**
 * @brief      Hybrid Velocity Obstacle Behavior
 *
 * A wrapper of the open-source implementation from
 * http://gamma.cs.unc.edu/HRVO/
 */
class HL_NAVIGATION_EXPORT HRVOBehavior : public Behavior,
                                          public GeometricState {
 public:
  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  HRVOBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
               float radius = 0.0f);
  ~HRVOBehavior();

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
  Vector2 compute_desired_velocity([[maybe_unused]] float time_step) override;

 private:
  GeometricState state;
  uint agentIndex;
  float rangeSq;
  std::unique_ptr<HRVO::Agent> _HRVOAgent;
  void add_neighbor(const Neighbor& neighbor, bool push_away = false,
                    float epsilon = 2e-3);
  void add_obstacle(const Disc& disc, bool push_away = false,
                    float epsilon = 2e-3);
  void prepare();

 private:
  inline static std::string type =
      Behavior::register_type<HRVOBehavior>("HRVO");
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_HRVO_H_
