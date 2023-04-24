/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_
#define HL_NAVIGATION_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_

#include <vector>

#include "hl_navigation_sim/state_estimation.h"
#include "hl_navigation_sim/world.h"
#include "hl_navigation_sim_export.h"

using hl_navigation::core::make_property;
using hl_navigation::core::Properties;
using hl_navigation::core::Property;

namespace hl_navigation::sim {

/**
 * @brief      Perfect state estimation within a range from the agent.
 *
 * *Properties*: range_of_view (float)
 */
struct HL_NAVIGATION_SIM_EXPORT BoundedStateEstimation
    : public StateEstimation {
  inline static const float default_range_of_view = 1.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_of_view_  The range of view
   */
  BoundedStateEstimation(float range_of_view_ = default_range_of_view)
      // float field_of_view_ = 0.0f,
      : StateEstimation(),
        // field_of_view(field_of_view_),
        range_of_view(range_of_view_) {}

  virtual ~BoundedStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range_of_view(float value) { range_of_view = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  float get_range_of_view() const { return range_of_view; }

  // void set_field_of_view(float v) { field_of_view = v; }

  // float get_field_of_view() const { return field_of_view; }

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties =
      Properties{
          // {"field_of_view", make_property<float, BoundedStateEstimation>(
          //                       &BoundedStateEstimation::get_field_of_view,
          //                       &BoundedStateEstimation::set_field_of_view,
          //                       0.0f, "Field of view (< 0 infinite)")},
          {"range_of_view",
           make_property<float, BoundedStateEstimation>(
               &BoundedStateEstimation::get_range_of_view,
               &BoundedStateEstimation::set_range_of_view,
               default_range_of_view, "Range of view (< 0 =infinite)")},
      } +
      StateEstimation::properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @brief      Gets the neighbors that lie within \ref get_range_of_view from the agent.
   *
   * @param[in]  agent  The agent
   * @param[in]  world  The world the agent is part of.
   *
   * @return     A list of neighbors around the agent 
   */
  virtual std::vector<Neighbor> neighbors_of_agent(const Agent *agent,
                                                   const World *world) const;

 protected:
#if 0
  BoundingBox bounding_box(const Agent *agent) const;

  virtual Neighbor perceive_neighbor(const Agent *agent,
                                     const Agent *neighbor) const;

  virtual bool visible(const Agent *agent, const Agent *neighbor) const;
#endif

  /**
   * @private
   */
  void update(Agent *agent, World *world) const override;

  /**
   * @private
   */
  void prepare(Agent *agent, World *world) const override;

  /**
   * @brief      Gets the geometric state of an agent.
   *
   * @private
   *
   * @return     The geometric state or ``nullptr`` if the agent behavior does
   * not have a environment state that is a subclass of \ref
   * hl_navigation::core::GeometricState
   */
  GeometricState *get_geometric_state(Agent *agent) const {
    if (agent) {
      if (Behavior *behavior = agent->get_behavior()) {
        return dynamic_cast<GeometricState *>(
            behavior->get_environment_state());
      }
    }
    return nullptr;
  }

 private:
  // float field_of_view;
  float range_of_view;
  inline const static std::string type =
      register_type<BoundedStateEstimation>("Bounded");
};

}  // namespace hl_navigation::sim

#endif /* end of include guard: \
          HL_NAVIGATION_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_ */
