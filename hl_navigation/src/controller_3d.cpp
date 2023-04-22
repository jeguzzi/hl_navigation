/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */
#include "hl_navigation/controller_3d.h"

namespace hl_navigation {

static bool overlaps(float a1, float a2, float b1, float b2) {
  return b1 <= a2 || a1 <= b2;
}

// TODO(J): filter becomes outdated when z or target z changes!!!

void Controller3::set_neighbors(const std::vector<Neighbor3>& neighbors) {
  if (!behavior) return;
  GeometricState* geometric_state =
      dynamic_cast<GeometricState*>(behavior->get_environment_state());
  if (!geometric_state) return;

  std::vector<Neighbor> neighbors_2d;
  for (const auto& cylinder : neighbors) {
    if (!limit_to_2d && altitude.value_set && cylinder.height > 0) {
      float z1, z2;
      if (altitude.target_set) {
        if (altitude.target < altitude.value) {
          z1 = altitude.target;
          z2 = altitude.value;
        } else {
          z2 = altitude.target;
          z1 = altitude.value;
        }
      } else {
        z1 = z2 = altitude.value;
      }
      if (!overlaps(cylinder.position[2],
                    cylinder.position[2] + cylinder.height, z1, z2)) {
        continue;
      }
    }
    neighbors_2d.push_back(cylinder.neighbor());
  }
  geometric_state->set_neighbors(neighbors_2d);
}

// TODO(Jerome): dry up
void Controller3::set_static_obstacles(const std::vector<Cylinder>& neighbors) {
  if (!behavior) return;
  GeometricState* geometric_state =
      dynamic_cast<GeometricState*>(behavior->get_environment_state());
  if (!geometric_state) return;
  std::vector<Disc> obstacles_2d;
  for (const auto& cylinder : neighbors) {
    if (!limit_to_2d && altitude.value_set && cylinder.height > 0) {
      float z1, z2;
      if (altitude.target_set) {
        if (altitude.target < altitude.value) {
          z1 = altitude.target;
          z2 = altitude.value;
        } else {
          z2 = altitude.target;
          z1 = altitude.value;
        }
      } else {
        z1 = z2 = altitude.value;
      }
      if (!overlaps(cylinder.position[2],
                    cylinder.position[2] + cylinder.height, z1, z2)) {
        continue;
      }
    }
    obstacles_2d.push_back(cylinder.disc());
  }
  geometric_state->set_static_obstacles(obstacles_2d);
}

}  // namespace hl_navigation
