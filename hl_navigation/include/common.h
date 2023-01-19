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

using CVector2 = Eigen::Vector2f;
using CRadians = float;
using Real = float;

CRadians inline polar_angle(const CVector2 & v) {
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

inline CVector2 unit(float angle) {
  return {cos(angle), sin(angle)};
}

inline CVector2 rotate(const CVector2 v, float angle) {
  Eigen::Rotation2D<float> rot(angle);
  return rot * v;
}

#endif
