#ifndef HL_NAVIGATION_BEHAVIOR_GEOMETRIC_H
#define HL_NAVIGATION_BEHAVIOR_GEOMETRIC_H value

#include "hl_navigation/common.h"

namespace hl_navigation {
/**
 * @brief A dynamic obstacle of circular shape.
 */
struct Disc {
  /**
   * The center of the disc in world frame
   */
  Vector2 position;
  /**
   * Velocity in world frame
   */
  Vector2 velocity;
  /**
   * Radius
   */
  float radius;
  /**
   * An additional softer margin to add to the obstacle.
   *
   * \warning We will probably replace \ref social_margin with a tag labeling
   * the type of obstacle and move the related property to \ref Behavior.
   */
  float social_margin;

  /**
   * @brief      Constructs a new instance.
   *
   * @param  position       The position
   * @param  radius         The radius
   * @param  social_margin  The social margin
   * @param  velocity       The velocity
   */
  Disc(const Vector2 &position, float radius, float social_margin = 0.0,
       const Vector2 velocity = Vector2::Zero())
      : position(position),
        velocity(velocity),
        radius(radius),
        social_margin(social_margin) {}
};

/**
 * @brief      A static obstacle of linear shape.
 */
struct LineSegment {
  /**
   * The position of the first vertex
   */
  Vector2 p1;
  /**
   * The position of the second vertex
   */
  Vector2 p2;
  /**
   * The unit vector along the segment
   */
  Vector2 e1;
  /**
   * The unit vector perpendicular to the segment. Oriented to the left with
   * respect to `e1`
   */
  Vector2 e2;
  /**
   * The segment length
   */
  float length;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  p1    The first vertex
   * @param[in]  p2    The second vertex
   */
  LineSegment(const Vector2 &p1, const Vector2 &p2)
      : p1(p1),
        p2(p2),
        e1((p2 - p1).normalized()),
        e2(-e1[1], e1[0]),
        length((p2 - p1).norm()) {}

  /**
   * @brief      Constructs a copy.
   *
   * @param[in]  segment  The segment to copy.
   */
  // LineSegment(const LineSegment & segment) : LineSegment(segment.p1,
  // segment.p2) {}
};

class GeometricState : protected RegisterChanges {
 public:
  GeometricState()
      : RegisterChanges(), static_obstacles(), neighbors(), line_obstacles() {}

  virtual ~GeometricState() = default;

  //----------- ENVIRONMENT STATE

  /**
   * @brief      Gets the current list of neighbors. Positions are in the world
   * fixed frame.
   *
   * @return     The neighbors.
   */
  const std::vector<Disc> &get_neighbors() { return neighbors; }
  /**
   * @brief      Sets the neighbors. Positions are in the world fixed frame.
   *
   * @param[in]  value
   */
  virtual void set_neighbors(const std::vector<Disc> &value) {
    neighbors = value;
    change(NEIGHBORS);
  }
  /**
   * @brief      Gets the current list of static obstacles. Positions are in the
   * world fixed frame.
   *
   * @return     The static obstacles
   */
  const std::vector<Disc> &get_static_obstacles() { return static_obstacles; }
  /**
   * @brief      Sets the static obstacles. Positions are in the world fixed
   * frame.
   *
   * @param[in]  value
   */
  virtual void set_static_obstacles(const std::vector<Disc> &value) {
    static_obstacles = value;
    change(STATIC_OBSTACLES);
  }
  /**
   * @brief      Gets the current list of line obstacles. Positions are in the
   * world fixed frame.
   *
   * @return     The line obstacles
   */
  const std::vector<LineSegment> &get_line_obstacles() {
    return line_obstacles;
  }
  /**
   * @brief      Sets the line obstacles. Positions are in the world fixed
   * frame.
   *
   * @param[in]  value
   */
  virtual void set_line_obstacles(const std::vector<LineSegment> &value) {
    line_obstacles = value;
    change(LINE_OBSTACLES);
  }

 protected:
  enum {
    NEIGHBORS = 1 << 0,
    STATIC_OBSTACLES = 1 << 1,
    LINE_OBSTACLES = 1 << 2
  };

 private:
  std::vector<Disc> static_obstacles;
  std::vector<Disc> neighbors;
  std::vector<LineSegment> line_obstacles;
};

}  // namespace hl_navigation

#endif  // HL_NAVIGATION_BEHAVIOR_GEOMETRIC_H