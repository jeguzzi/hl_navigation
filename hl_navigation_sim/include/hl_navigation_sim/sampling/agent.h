/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_AGENT_H
#define HL_NAVIGATION_SIM_AGENT_H

#include <iostream>
#include <memory>

#include "hl_navigation_sim/sampling/register.h"
#include "hl_navigation_sim/scenario.h"
#include "hl_navigation_sim/world.h"

namespace hl_navigation_sim {

template <typename W = World>
struct AgentSampler : public Sampler<typename W::A::C>,
                      virtual public Scenario::Group {
  using A = typename W::A;
  using C = typename A::C;
  using B = typename A::B;
  using K = typename A::K;
  using T = typename A::T;
  using S = typename A::S;
  using Sampler<C>::sample;

  explicit AgentSampler() : Sampler<C>(), number{0} {}

  void add_to_world(World* world) override {
    if (W* w = dynamic_cast<W*>(world)) {
      for (unsigned i = 0; i < number; ++i) {
        w->add_agent(sample());
      }
    } else {
      std::cerr << "Trying to add agent sampler to wrong World type"
                << std::endl;
    }
  }

 protected:
  C s() override {
    C c = A::make(radius ? radius->sample() : 0.0f, behavior.sample(),
                  kinematic.sample(), task.sample(), state_estimation.sample(),
                  control_period ? control_period->sample() : 0.0f);
    A* agent = get<A, C>::ptr(c);
    agent->pose = {{x ? x->sample() : 0.0f, y ? y->sample() : 0.0f},
                   theta ? theta->sample() : 0.0f};
    return c;
  }

 public:
  void reset() override {
    Sampler<C>::reset();
    behavior.reset();
    kinematic.reset();
    task.reset();
    state_estimation.reset();
    if (x) x->reset();
    if (y) y->reset();
    if (theta) theta->reset();
    if (radius) radius->reset();
    if (control_period) control_period->reset();
  }

  BehaviorSampler<B> behavior;
  KinematicSampler<K> kinematic;
  SamplerFromRegister<T> task;
  SamplerFromRegister<S> state_estimation;
  std::shared_ptr<Sampler<float>> x;
  std::shared_ptr<Sampler<float>> y;
  std::shared_ptr<Sampler<float>> theta;
  std::shared_ptr<Sampler<float>> radius;
  std::shared_ptr<Sampler<float>> control_period;
  unsigned number;
};

}  // namespace hl_navigation_sim

#endif  // HL_NAVIGATION_SIM_SAMPLER_H
