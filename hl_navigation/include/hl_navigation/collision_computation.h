/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef HL_NAVIGATION_COLLISION_COMPUTATION_H_
#define HL_NAVIGATION_COLLISION_COMPUTATION_H_

#include <algorithm>
#include <vector>

#include "hl_navigation/common.h"
#include "hl_navigation/state/geometric.h"

namespace hl_navigation {

struct DiscCache {
  Vector2 delta;
  Vector2 velocity;
  float distance;
  float C;
  Radians gamma;
  Radians visible_angle;

  DiscCache(Vector2 delta, float margin, Vector2 velocity = Vector2::Zero());
};

/**
 * @brief      TODO
 *
 */
class CollisionComputation {
 public:
  using CollisionMap = std::vector<std::tuple<float, float>>;

  static inline std::vector<LineSegment> empty = {};

  CollisionComputation()
      : line_obstacles(empty),
        neighbors_cache(),
        static_obstacles_cache(),
        pose(),
        margin(0.0) {}

  CollisionMap get_free_distance_for_sector(Radians from, Radians length,
                                            size_t resolution,
                                            float max_distance,
                                            bool dynamic = false,
                                            float speed = 0.0f);

  void setup(Pose2 pose_, float margin_,
             const std::vector<LineSegment> &line_segments,
             std::vector<DiscCache> static_discs,
             std::vector<DiscCache> dynamic_discs) {
    line_obstacles = line_segments;
    static_obstacles_cache = static_discs;
    neighbors_cache = dynamic_discs;
    pose = pose_;
    margin = margin_;
  }

  void setup(Pose2 pose_, float margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<Disc> &static_discs,
             const std::vector<Disc> &dynamic_discs) {
    line_obstacles = line_segments;
    pose = pose_;
    margin = margin_;
    neighbors_cache.clear();
    neighbors_cache.reserve(dynamic_discs.size());
    for (const auto &disc : dynamic_discs) {
      neighbors_cache.push_back({disc.position - pose.position,
                                 margin_ + disc.radius, disc.velocity});
    }
    static_obstacles_cache.clear();
    static_obstacles_cache.reserve(static_discs.size());
    for (const auto &disc : static_discs) {
      static_obstacles_cache.push_back(
          {disc.position - pose.position, margin_ + disc.radius});
    }
  }

  // angle is absolute
  float static_free_distance(Radians angle, float max_distance,
                             bool include_neighbors = true);
  // angle is absolute
  float dynamic_free_distance(Radians angle, float max_distance, float speed);
  bool dynamic_may_collide(const DiscCache &c, float max_distance, float speed);
  bool static_may_collide(const DiscCache &c, float max_distance);

 protected:
  // Should be a ref to avoid copies
  std::vector<LineSegment> &line_obstacles;
  std::vector<DiscCache> neighbors_cache;
  std::vector<DiscCache> static_obstacles_cache;
  Pose2 pose;
  float margin;

  /**
   * Marks absence of collisions for an entry in \ref
   * get_collision_distance_cache
   */
  static constexpr int no_collision = -1;
  float static_free_distance_to(const LineSegment &line, Radians alpha);

  float static_free_distance_to(const DiscCache &disc, Radians alpha);

  float dynamic_free_distance_to(const DiscCache &disc, Radians alpha,
                                 float speed);

  template <typename T>
  float static_free_distance_to_collection(Radians angle, float max_distance,
                                           const std::vector<T> &objects) {
    float min_distance = max_distance;
    for (const auto &object : objects) {
      float distance = static_free_distance_to(object, angle);
      if (distance < 0) continue;
      min_distance = fmin(min_distance, distance);
      if (min_distance == 0) return 0;
    }
    return min_distance;
  }

  template <typename T>
  float dynamic_free_distance_to_collection(Radians angle, float max_distance,
                                            float speed,
                                            const std::vector<T> &objects) {
    float min_distance = max_distance;
    for (const auto &object : objects) {
      float distance = dynamic_free_distance_to(object, angle, speed);
      if (distance < 0) continue;
      min_distance = fmin(min_distance, distance);
      if (min_distance == 0) return 0;
    }
    return min_distance;
  }
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_COLLISION_COMPUTATION_H_
