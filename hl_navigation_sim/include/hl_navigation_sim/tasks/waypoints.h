/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_TASKS_WAYPOINTS_H_
#define HL_NAVIGATION_SIM_TASKS_WAYPOINTS_H_

#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation_sim/task.h"
#include "hl_navigation_sim_export.h"

using hl_navigation::Properties;
using hl_navigation::Property;
using hl_navigation::make_property;
using hl_navigation::Vector2;

namespace hl_navigation_sim {

/**
 * A sequence of points to reach.
 */
using Waypoints = std::vector<hl_navigation::Vector2>;

/**
 * @brief      This class implement a task that makes the agent reach a sequence
 * of waypoints, calling \ref hl_navigation::Controller::go_to_position for the
 * next waypoint after the current has been reached within a tolerance.
 * 
 * The task notifies when a new waypoint is set by calling a callback.
 * 
 * *Properties*: waypoints (list of \ref hl_navigation::Vector2), loop (bool), tolerance (float)
 */
struct HL_NAVIGATION_SIM_EXPORT WaypointsTask : Task {
  /**
   * A callback with argument the current waypoint.
   */
  // using Callback = std::function<void(const Vector2 &)>;

  /**
   * Whether by default the task loops.
   */
  inline static const bool default_loop = true;
  /**
   * The default goal tolerance.
   */
  inline static const bool default_tolerance = 1.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  waypoints_  The waypoints
   * @param[in]  loop_       Whether it should start from begin after reaching the last
   * waypoint
   * @param[in]  tolerance_  The goal tolerance applied to each waypoint.
   */
  WaypointsTask(Waypoints waypoints_ = {}, bool loop_ = default_loop,
                float tolerance_ = default_tolerance)
      : Task(),
        waypoints(waypoints_),
        waypoint(waypoints.begin()),
        loop(loop_),
        tolerance(tolerance_){}

  virtual ~WaypointsTask() = default;


  /**
   * @private
   */
  bool done() const override;

  /**
   * @brief      Sets the waypoints.
   *
   * @param[in]  value  The desired waypoints
   */
  void set_waypoints(const Waypoints &value) {
    waypoints = value;
    waypoint = waypoints.begin();
  }
  /**
   * @brief      Sets the goal tolerance applied to each waypoint.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(float value) { tolerance = std::max(value, 0.0f); }
  /**
   * @brief      Sets whether it should start from begin after reaching the last waypoint
   *
   * @param[in]  value  The desired value
   */
  void set_loop(bool value) { loop = value; }
  /**
   * @brief      Gets the waypoints.
   *
   * @return     The waypoints.
   */
  Waypoints get_waypoints() const { return waypoints; }
  /**
   * @brief      Gets the goal tolerance applied to each waypoint.
   *
   * @return     The tolerance.
   */
  float get_tolerance() const { return tolerance; }
  /**
   * @brief      Gets whether it should start from begin after reaching the last waypoint.
   *
   * @return     True if it should loop.
   */
  float get_loop() const { return loop; }
  /**
   * @brief      Gets the properties.
   *
   * @private
   * @return     The properties.
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties = Properties{
      {"waypoints",
       make_property<Waypoints, WaypointsTask>(&WaypointsTask::get_waypoints,
                                               &WaypointsTask::set_waypoints,
                                               Waypoints{}, "waypoints")},
      {"loop", make_property<bool, WaypointsTask>(&WaypointsTask::get_loop,
                                                  &WaypointsTask::set_loop,
                                                  default_loop, "loop")},
      {"tolerance",
       make_property<float, WaypointsTask>(&WaypointsTask::get_tolerance,
                                           &WaypointsTask::set_tolerance,
                                           default_tolerance, "tolerance")},
  };

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 protected:
  /**
   * @private
   */
  void update(Agent *agent, World * world, float time) override;

 private:
  Waypoints waypoints;
  Waypoints::iterator waypoint;
  bool loop;
  float tolerance;
  // std::vector<Callback> callbacks;
  bool running;
  inline const static std::string type =
      register_type<WaypointsTask>("Waypoints");
};

}  // namespace hl_navigation_sim

#endif /* end of include guard: HL_NAVIGATION_SIM_TASKS_WAYPOINTS_H_ */
