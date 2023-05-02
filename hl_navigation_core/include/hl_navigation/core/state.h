/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_CORE_STATE_H_
#define HL_NAVIGATION_CORE_STATE_H_

namespace hl_navigation::core {

struct EnvironmentState {
  EnvironmentState() = default;
  virtual ~EnvironmentState() = default;
};

}  // namespace hl_navigation::core

#endif  // HL_NAVIGATION_CORE_STATE_H_
