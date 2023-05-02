/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "hl_navigation/core/collision_computation.h"

using hl_navigation::core::CollisionComputation;
using hl_navigation::core::Disc;
using hl_navigation::core::LineSegment;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
  CollisionComputation f;
  f.setup({}, 0.0, {}, {Disc{{2.0, 0.0}, 1.0}}, {});
  for (const auto& [a, d] :
       f.get_free_distance_for_sector(-1.0, 2.0, 3, 3.0, false)) {
    printf("\n%.3f: %.3f\n", a, d);
  }
  return 0;
}
