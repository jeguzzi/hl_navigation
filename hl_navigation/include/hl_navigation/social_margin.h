#ifndef HL_NAVIGATION_SOCIAL_MARGIN_H
#define HL_NAVIGATION_SOCIAL_MARGIN_H

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>

class SocialMargin {
 public:
  enum class ModulationType { zero, constant, linear, quadratic, logistic };

  class Modulation {
   public:
    virtual ~Modulation() = default;
    Modulation(){};
    virtual float operator()(float margin,
                             [[maybe_unused]] float distance) const {
      return margin;
    }
    virtual float operator()(float margin) const { return margin; }
  };

  class ZeroModulation : public Modulation {
   public:
    using Modulation::Modulation;
    float operator()([[maybe_unused]] float margin,
                     [[maybe_unused]] float distance) const override {
      return 0.0f;
    }
    float operator()([[maybe_unused]] float margin) const override {
      return 0.0f;
    }
  };

  class ConstantModulation : public Modulation {
   public:
    using Modulation::Modulation;
  };

  // returns margin above upper_distance, and a linear interpolation below
  class LinearModulation : public Modulation {
   public:
    explicit LinearModulation(float upper_distance)
        : Modulation(), upper_distance(upper_distance) {}
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      float upper = std::max(upper_distance, margin);
      if (distance > upper) return margin;
      return margin * distance / upper;
    }

    float get_upper_distance() const { return upper_distance; }

   private:
    float upper_distance;
  };

  // returns margin above upper_distance, and a quadratic interpolation below
  class QuadraticModulation : public Modulation {
   public:
    explicit QuadraticModulation(float upper_distance)
        : Modulation(), upper_distance(upper_distance) {}
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      const float upper = std::max(upper_distance, 2 * margin);
      if (distance > upper) return margin;
      const float x = distance / upper;
      return -margin * x * x + 2 * margin * x;
    }

    float get_upper_distance() const { return upper_distance; }

   private:
    float upper_distance;
  };

  class LogisticModulation : public Modulation {
   public:
    using Modulation::Modulation;
    float operator()(float margin, float distance) const override {
      if (distance < 0) return 0.0;
      return 2 * distance + margin -
             margin * std::log2(1 + std::exp2(2 * distance / margin));
    }
  };

  explicit SocialMargin(float value = 0.0f)
      : default_social_margin(std::max(0.0f, value)),
        social_margins(),
        modulation(std::make_shared<ConstantModulation>()) {}

  std::shared_ptr<Modulation> get_modulation() const { return modulation; }
  void set_modulation(const std::shared_ptr<Modulation>& value) {
    if (value) {
      modulation = value;
    }
  }

  // float get_upper_distance() const { return upper_distance; }
  // void set_upper_distance(float value) {
  //   upper_distance = std::max(0.0f, value);
  // }

  float get() const { return (*modulation)(default_social_margin); }
  void set(float value) { default_social_margin = std::max(0.0f, value); }
  float get(unsigned type) { return (*modulation)(get_value(type)); }
  void set(unsigned type, float value) {
    social_margins[type] = std::max(0.0f, value);
  }

  float get(unsigned type, float distance) {
    return (*modulation)(get_value(type), distance);
  }

  float get_value(unsigned type) {
    return social_margins[type].value_or(default_social_margin);
  }

  float get_default_value() const { return default_social_margin; }

  const std::map<unsigned, std::optional<float>>& get_values() const {
    return social_margins;
  }

  // distance is distance between two discs (not between centers)
  // return the margin to add to the safety margin!
  // float get_linear(unsigned type, float distance, float upper_distance) {
  //   return linear_margin(get(type), distance, upper_distance);
  // }

  // float get_quadratic(unsigned type, float distance, float upper_distance) {
  //   return quadratic_margin(get(type), distance, upper_distance);
  // }

  // float get_logistic(unsigned type, float distance) {
  //   return logistic_margin(get(type), distance);
  // }

  // // returns margin above upper_distance, and a linear interpolation below
  // static float linear_margin(float margin, float distance,
  //                            float upper_distance) {
  //   if (distance < 0) return 0.0;
  //   upper_distance = std::max(upper_distance, margin);
  //   if (distance > upper_distance) return margin;
  //   return margin * upper_distance / distance;
  // }

  // // returns margin above upper_distance, and a quadratic interpolation below
  // static float quadratic_margin(float margin, float distance,
  //                               float upper_distance) {
  //   if (distance < 0) return 0.0;
  //   upper_distance = std::max(upper_distance, 2 * margin);
  //   if (distance > upper_distance) return margin;
  //   const float x = distance / upper_distance;
  //   return -margin * x * x + 2 * margin * x;
  // }

  // static float logistic_margin(float margin, float distance) {
  //   if (distance < 0) return 0.0;
  //   return 2 * distance + margin -
  //          margin * std::log2(1 + std::exp2(2 * distance / margin));
  // }

 private:
  float default_social_margin;
  std::map<unsigned, std::optional<float>> social_margins;
  std::shared_ptr<Modulation> modulation;
};

#endif  // HL_NAVIGATION_SOCIAL_MARGIN_H
