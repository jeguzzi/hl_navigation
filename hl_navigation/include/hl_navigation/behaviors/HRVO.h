/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_BEHAVIOR_HRVO_H_
#define HL_NAVIGATION_BEHAVIOR_HRVO_H_

#include <memory>

#include "hl_navigation/behavior.h"

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
class HRVOBehavior : public Behavior {
 public:
  HRVOBehavior(std::shared_ptr<Kinematic> kinematic, float radius);
  ~HRVOBehavior();

 protected:
  Vector2 compute_desired_velocity() override;

  void add_neighbor(const Disc& disc);
  void prepare();

 private:
  uint agentIndex;
  float rangeSq;
  std::unique_ptr<HRVO::Agent> _HRVOAgent;
  static const char* name;
  bool cache_is_valid() const;
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_HRVO_H_
