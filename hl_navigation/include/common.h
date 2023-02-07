/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdio.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef unsigned int uint;

namespace hl_navigation {

using Vector2 = Eigen::Vector2f;
using Radians = float;

Radians inline polar_angle(const Vector2 & v) {
  return std::atan2(v.y(), v.x());
}

float inline normalize(float v) {
  v = std::fmod(v, 2 * M_PI);
  if (v < -M_PI) {
    v += 2 * M_PI;
  } else if (v > M_PI) {
    v -= 2 * M_PI;
  }
  return v;
}

inline Vector2 unit(float angle) {
  return {cos(angle), sin(angle)};
}

inline Vector2 rotate(const Vector2 v, float angle) {
  Eigen::Rotation2D<float> rot(angle);
  return rot * v;
}

}

#endif
