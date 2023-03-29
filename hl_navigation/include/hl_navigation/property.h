#ifndef HL_NAVIGATION_PROPERTY_H
#define HL_NAVIGATION_PROPERTY_H

#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "./common.h"
#include "./utilities.h"

namespace hl_navigation {

struct HasProperties;

struct Property {
  using Field =
      std::variant<bool, int, float, std::string, Vector2, std::vector<bool>,
                   std::vector<int>, std::vector<float>,
                   std::vector<std::string>, std::vector<Vector2>>;
  using Getter = std::function<Field(const HasProperties*)>;
  using Setter = std::function<void(HasProperties*, const Field&)>;

  Getter getter;
  Setter setter;
  Field default_value;
  std::string type_name;
  std::string description;
  std::string owner_type_name;
};

using Properties = std::map<std::string, Property>;

inline Properties operator+(const Properties& p1, const Properties& p2) {
  std::map<std::string, Property> r = p1;
  for (const auto& [k, v] : p2) {
    r.emplace(std::make_tuple(k, v));
  }
  return r;
}

template <typename T, class C>
using TypedGetter = std::function<T(const C*)>;

template <typename T, class C>
using TypedSetter = std::function<void(C*, const T&)>;

template <typename T, class C>
inline Property make_property(const TypedGetter<T, C>& getter,
                              const TypedSetter<T, C>& setter,
                              const T& default_value,
                              const std::string& description = "") {
  Property property;
  property.description = description;
  property.default_value = default_value;
  property.type_name = std::string(get_type_name<T>());
  property.owner_type_name = std::string(get_type_name<C>());
  property.getter = [getter](const HasProperties* obj) {
    const auto C_obj = dynamic_cast<const C*>(obj);
    if (!C_obj) throw std::bad_cast();
    return getter(C_obj);
  };
  property.setter = [setter](HasProperties* obj, const Property::Field& value) {
    auto C_obj = dynamic_cast<C*>(obj);
    if (!C_obj) return;
    std::visit(
        [&setter, &C_obj](auto&& arg) {
          using V = std::decay_t<decltype(arg)>;
          if constexpr (std::is_convertible<V, T>::value) {
            setter(C_obj, static_cast<T>(arg));
          }
        },
        value);
  };
  return property;
}

struct HasProperties {
  virtual ~HasProperties() = default;

  static inline std::map<std::string, Property> properties = Properties{};

  virtual const Properties& get_properties() const { return properties; };

  void set(const std::string& name, const Property::Field& value) {
    const auto& properties = get_properties();
    if (properties.count(name)) {
      const Property& property = properties.at(name);
      if (property.setter) {
        property.setter(this, value);
      }
    }
  }

  template <typename V>
  void set_value(const std::string& name, const V& value) {
    const auto& properties = get_properties();
    if (properties.count(name)) {
      const Property& property = properties.at(name);
      if (!property.setter) return;
      std::visit(
          [&property, this, &value](auto&& arg) {
            using T = std::decay_t<decltype(arg)>;
            if constexpr (std::is_convertible<V, T>::value) {
              std::cout << get_type_name<V>() << "=>" << get_type_name<T>()
                        << std::endl;
              property.setter(this, static_cast<T>(value));
            }
          },
          property.default_value);
    }
  }

  Property::Field get(const std::string& name) const {
    const auto& properties = get_properties();
    if (properties.count(name)) {
      const Property& property = properties.at(name);
      if (property.getter) {
        return property.getter(this);
      }
    }
    throw std::runtime_error("No property " + name);
  }
};

}  // namespace hl_navigation

// namespace Eigen {

// inline std::ostream& operator<<(
//     std::ostream& os, const std::vector<Matrix<float, 2, 1, 0>>& items) {
//   os << "[";
//   bool first = true;
//   for (const auto& item : items) {
//     if (!first) os << ", ";
//     os << "[" << item[0] << ", " << item[1] << "]";
//     first = false;
//   }
//   os << "]";
//   return os;
// }
// }  // namespace Eigen


template <typename T>
inline std::ostream& operator<<(std::ostream& os,
                                const std::vector<T>& values) {
  os << "[";
  bool f = true;
  for (const auto& value : values) {
    if (!f) os << ", ";
    f = false;
    os << value;
  }
  os << "]";
  return os;
}


inline std::ostream& operator<<(std::ostream& os,
                                const hl_navigation::Property::Field& value) {
  std::visit([&os](auto&& arg) { os << arg; }, value);
  return os;
}

#endif  // HL_NAVIGATION_PROPERTY_H
