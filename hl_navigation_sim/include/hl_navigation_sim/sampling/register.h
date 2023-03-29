/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_REGISTER_H
#define HL_NAVIGATION_SIM_REGISTER_H

#include <iostream>
#include <memory>
#include <random>

#include "hl_navigation/behavior.h"
#include "hl_navigation/kinematic.h"
#include "hl_navigation_sim/sampling/sampler.h"
#include "hl_navigation_sim/world.h"

using hl_navigation::Behavior;
using hl_navigation::Kinematic;

namespace hl_navigation_sim {

template <typename T, typename C>
struct get {
  static T* ptr(const C&);
};

template <typename T>
struct get<T, std::shared_ptr<T>> {
  static T* ptr(const std::shared_ptr<T>& c) { return c.get(); }
};

template <typename T>
struct SamplerFromRegister : public Sampler<typename T::C> {
  using C = typename T::C;

  explicit SamplerFromRegister(const std::string& type = "")
      : Sampler<C>(), type(type), properties() {}

  typename T::C s() override {
    C c = T::make_type(type);
    T* t = get<T, C>::ptr(c);
    if (!t) {
      // std::cout << "Unknown type " << type << std::endl;
      return c;
    }
    for (const auto& [name, property] : properties) {
      if (property) {
        auto value = property->sample();
        t->set(name, value);
      }
    }
    return c;
  }

  void reset() override {
    Sampler<C>::reset();
    for (auto& [k, v] : properties) {
      if (v) v->reset();
    }
  }

  std::string type;
  std::map<std::string, std::shared_ptr<PropertySampler>> properties;
};

template <typename T = Behavior>
struct BehaviorSampler : public SamplerFromRegister<T> {
  using C = typename T::C;

  explicit BehaviorSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type) {}

 protected:
  C s() override {
    C c = SamplerFromRegister<T>::s();
    T* behavior = get<T, C>::ptr(c);
    if (!behavior) return c;
    if (optimal_speed) {
      behavior->set_optimal_speed(optimal_speed->sample());
    }
    if (optimal_angular_speed) {
      behavior->set_optimal_angular_speed(optimal_angular_speed->sample());
    }
    if (horizon) {
      behavior->set_horizon(horizon->sample());
    }
    if (rotation_tau) {
      behavior->set_rotation_tau(rotation_tau->sample());
    }
    if (safety_margin) {
      behavior->set_safety_margin(safety_margin->sample());
    }
    return c;
  }

 public:
  void reset() override {
    SamplerFromRegister<T>::reset();
    if (optimal_speed) optimal_speed->reset();
    if (optimal_angular_speed) optimal_angular_speed->reset();
    if (rotation_tau) rotation_tau->reset();
    if (safety_margin) safety_margin->reset();
    if (horizon) horizon->reset();
  }

  std::shared_ptr<Sampler<float>> optimal_speed;
  std::shared_ptr<Sampler<float>> optimal_angular_speed;
  std::shared_ptr<Sampler<float>> rotation_tau;
  std::shared_ptr<Sampler<float>> safety_margin;
  std::shared_ptr<Sampler<float>> horizon;
};

template <typename T = Kinematic>
struct KinematicSampler : public SamplerFromRegister<T> {
  explicit KinematicSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type) {}

  using C = typename T::C;

  C s() override {
    C c = SamplerFromRegister<T>::s();
    T* kinematic = get<T, C>::ptr(c);
    if (!kinematic) return c;
    if (max_speed) {
      kinematic->set_max_speed(max_speed->sample());
    }
    if (max_angular_speed) {
      kinematic->set_max_angular_speed(max_angular_speed->sample());
    }
    return c;
  }

  void reset() override {
    SamplerFromRegister<T>::reset();
    if (max_speed) max_speed->reset();
    if (max_angular_speed) max_angular_speed->reset();
  }

  std::shared_ptr<Sampler<float>> max_speed;
  std::shared_ptr<Sampler<float>> max_angular_speed;
};

template <typename T = Task>
using TaskSampler = SamplerFromRegister<T>;
template <typename T = StateEstimation>
using StateEstimationSampler = SamplerFromRegister<T>;

}  // namespace hl_navigation_sim

#endif  // HL_NAVIGATION_SIM_REGISTER_H
