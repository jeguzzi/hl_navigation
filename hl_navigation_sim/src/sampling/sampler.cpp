#include "hl_navigation/sim/sampling/sampler.h"

namespace hl_navigation::sim {

static unsigned _seed = 0;
static std::default_random_engine _generator(_seed);

// Check that 0 is the current value;

std::default_random_engine & random_generator() {
  return _generator;
}

void set_random_seed(unsigned seed) { 
  _seed = seed;
  _generator.seed(seed); 
}

unsigned get_random_seed() {
  return _seed;
}

}  // namespace hl_navigation::sim
