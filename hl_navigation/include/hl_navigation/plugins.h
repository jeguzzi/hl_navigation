#ifndef HL_NAVIGATION_PLUGINS_H
#define HL_NAVIGATION_PLUGINS_H

#include <iostream>
#include <sstream>
#include <vector>

// TODO(Jerome): add windows
#include <dlfcn.h>

#include <cstdlib>

static inline std::string default_plugins_env_name = "HL_NAVIGATION_PLUGINS";

inline std::vector<std::string> get_plugins(
    const std::string& name = default_plugins_env_name) {
  char* env = getenv(name.c_str());
  if (!env) return {};
  std::vector<std::string> paths;
  std::istringstream f(env);
  std::string s;
  while (std::getline(f, s, ':')) {
    paths.push_back(s);
  }
  return paths;
}

inline void load_plugins(const std::string& name = default_plugins_env_name) {
  for (const auto& path : get_plugins(name)) {
    void* lib = dlopen(path.c_str(), RTLD_LAZY);
    if (lib) {
      std::cout << "Loaded " << path << std::endl;
    }
  }
}

#endif  // HL_NAVIGATION_PLUGINS_H
