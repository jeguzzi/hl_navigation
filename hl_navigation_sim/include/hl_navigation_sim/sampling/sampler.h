/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_SIM_SAMPLING_SAMPLER_H
#define HL_NAVIGATION_SIM_SAMPLING_SAMPLER_H

#include <iostream>
#include <random>
#include <vector>

#include "hl_navigation/property.h"

namespace hl_navigation_sim {

template <typename T>
struct Sampler {
  Sampler() {}
  virtual ~Sampler() = default;
  T sample() {
    T v = s();
    samples++;
    return v;
  }
  unsigned count() const { return samples; }
  friend std::ostream& operator<<(std::ostream& os, const Sampler& c) {
    return c.output(os);
  }

  virtual void reset() { samples = 0; }

  virtual T s() = 0;
  virtual std::ostream& output(std::ostream& os) const {
    return os << "Sampler";
  }

  unsigned samples;
};

template <typename T>
struct Constant final : public Sampler<T> {
  explicit Constant(T value) : Sampler<T>{}, value{value} {}

  T s() override { return value; }
  std::ostream& output(std::ostream& os) const override {
    return os << "Constant(" << value << ")";
  }

  T value;
};

template <typename T>
struct Regular final : public Sampler<T> {
  explicit Regular(const T& start, const T& end, unsigned number)
      : Sampler<T>{},
        start{start},
        step{number > 0 ? (end - start) / (number - 1) : 0.0f},
        number{number} {}

  using Sampler<T>::count;

  T s() override { return start + step * (count() % number); }
  std::ostream& output(std::ostream& os) const override {
    return os << "Regular<" << start << " + " << step << ">";
  }

  T start;
  T step;
  unsigned number;
};

template <typename T>
struct Sequence final : public Sampler<T> {
  explicit Sequence(const std::vector<T>& values)
      : Sampler<T>{}, values{values} {}

  using Sampler<T>::count;

  T s() override {
    if (values.size()) {
      const size_t i = count() % values.size();
      return values[i];
    }
    std::cerr << "No samples in " << *this << std::endl;
    return T{};
  }
  std::ostream& output(std::ostream& os) const override {
    os << "Sequence({";
    for (const auto& item : values) {
      os << item << ", ";
    }
    os << "})";
    return os;
  }

  std::vector<T> values;
};

using PropertySampler = Sampler<hl_navigation::Property::Field>;

}  // namespace hl_navigation_sim

#endif  // HL_NAVIGATION_SIM_SAMPLING_SAMPLER_H
