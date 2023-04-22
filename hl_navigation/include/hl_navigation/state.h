/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_STATE_H_
#define HL_NAVIGATION_STATE_H_

namespace hl_navigation {

struct EnvironmentState {
  EnvironmentState() = default;
  virtual ~EnvironmentState() = default;
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_STATE_H_
