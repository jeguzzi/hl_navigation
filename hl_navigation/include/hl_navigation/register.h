#ifndef HL_NAVIGATION_REGISTER_H
#define HL_NAVIGATION_REGISTER_H

#include "./property.h"
#include "./utilities.h"

namespace hl_navigation {

// T must define
// - static properties: Properties T::properties
template <typename T>
struct HasRegister {
  using C = std::shared_ptr<T>;
  using Factory = std::function<C()>;

  static std::map<std::string, Factory> &factory() {
    static std::map<std::string, Factory> f;
    return f;
  };

  static std::map<std::string, Properties> &type_properties() {
    static std::map<std::string, Properties> p;
    return p;
  };

  static C make_type(const std::string &type) {
    if (factory().count(type)) {
      return factory()[type]();
    }
    // std::cerr << "Type " << type << " is not a registered " << get_type_name<T>()
    //           << std::endl;
    return nullptr;
  }

  /**
   * @brief      Returns the types of all registered classes
   *
   * @return     The registered classes.
   */
  static std::vector<std::string> types() {
    std::vector<std::string> keys;
    std::transform(factory().begin(), factory().end(), back_inserter(keys),
                   [](std::pair<std::string, Factory> p) { return p.first; });
    return keys;
  }

  template <typename S>
  static std::string register_type(const std::string &type) {
    // std::cout << "register_type " << get_type_name<S>() << " as " << type << std::endl;
    static_assert(std::is_base_of_v<T, S>);
    if (factory().count(type)) {
      // std::cerr << "Type " << type << " already registered for "
      //           << get_type_name<S>() << std::endl;
    } else {
      factory()[type] = []() {
        return std::make_shared<S>();
      };
      type_properties()[type] = S::properties;
    }

    return type;
  }

  virtual std::string get_type() const { return type; }

  static inline const std::string type = "";
};

}

#endif  // HL_NAVIGATION_REGISTER_H
