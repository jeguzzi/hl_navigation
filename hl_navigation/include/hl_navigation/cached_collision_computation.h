/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_CACHED_COLLISION_COMPUTATION_H_
#define HL_NAVIGATION_CACHED_COLLISION_COMPUTATION_H_

#include <algorithm>
#include <array>
#include <vector>

#include "hl_navigation/behavior.h"
#include "hl_navigation/collision_computation.h"
#include "hl_navigation/common.h"

namespace hl_navigation {

/**
 * @brief      TODO
 *
 */
class CachedCollisionComputation : public CollisionComputation {
 public:
  /**
   * Marks yet-not-computed entries in \ref get_cache
   */
  static constexpr int uncomputed = -2;

  CachedCollisionComputation()
      : CollisionComputation(),
        from_relative_angle(0.0),
        length(0.0),
        resolution(0),
        speed(0.0),
        max_distance(0.0),
        dynamic_cache(),
        static_cache() {}

  void set_resolution(size_t value) {
    if (value > 0 && value != resolution) {
      resolution = value;
      dynamic_cache.resize(value);
      for (auto &s : static_cache) {
        s.resize(value);
      }
      reset();
    }
  }

  size_t get_resolution() const { return resolution; }

  void set_min_angle(float value) {
    value = normalize(value);
    if (value != from_relative_angle) {
      from_relative_angle = value;
      reset();
    }
  }

  float get_min_angle() const { return from_relative_angle; }

  void set_length(float value) {
    if (value > 0) {
      value = std::min<float>(value, 2 * M_PI);
      if (length != value) {
        length = value;
        reset();
      }
    }
  }

  float get_length() const { return length; }

  void set_max_distance(float value) {
    if (value > 0 && value != max_distance) {
      max_distance = value;
      reset();
    }
  }

  float get_max_distance() const { return max_distance; }

  void set_speed(float value) {
    if (value > 0 and value != speed) {
      speed = value;
    }
    std::fill(dynamic_cache.begin(), dynamic_cache.end(), uncomputed);
  }

  float get_speed() const { return speed; }

  // angle is absolute
  float static_free_distance(Radians angle, bool include_neighbors = true);
  // angle is absolute
  float dynamic_free_distance(Radians angle);

  void setup(Pose2 pose_, float margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<DiscCache> &static_discs,
             const std::vector<DiscCache> &dynamic_discs) {
    CollisionComputation::setup(pose_, margin_, line_segments, static_discs,
                                dynamic_discs);
    reset();
  }

  void setup(Pose2 pose_, float margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<Disc> &static_discs,
             const std::vector<Neighbor> &dynamic_discs) {
    CollisionComputation::setup(pose_, margin_, line_segments, static_discs,
                                dynamic_discs);
    reset();
  }

  void reset() {
    std::fill(static_cache[false].begin(), static_cache[false].end(),
              uncomputed);
    std::fill(static_cache[true].begin(), static_cache[true].end(), uncomputed);
    std::fill(dynamic_cache.begin(), dynamic_cache.end(), uncomputed);
  }

  /**
   * @brief      Gets a pointer where collision distances are cached. Similar to
   * \ref get_collision_distance but no effort is made to compute entry not
   * already computed by \ref cmd_twist. Negative entries means:
   *               - \ref uncomputed = -2 -> distance not computed
   *               - \ref NO_COLLISION = -1 -> no collision (up to \ref
   * get_horizon)
   *
   * @param[in]  assuming_static  The assuming static
   *
   * @return     The collision distance cache.
   */
  const std::vector<float> &get_cache(bool assuming_static = false,
                                      bool include_neighbors = true) {
    if (assuming_static) return static_cache[include_neighbors];
    return dynamic_cache;
  }

  CollisionMap get_free_distance(bool dynamic = false);

 private:
  float from_relative_angle;
  float length;
  size_t resolution;
  float speed;
  float max_distance;
  std::vector<float> dynamic_cache;
  std::array<std::vector<float>, 2> static_cache;

  // can be outsize of 0 ... resolution range
  int index_of(Radians delta_relative_angle) {
    if (resolution < 2) return 0;
    return (delta_relative_angle - from_relative_angle) / length *
           static_cast<float>(resolution - 1);
  }
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_CACHED_COLLISION_COMPUTATION_H_
