#pragma once

#include <ostream>
#include <sstream>
#include <string>

#include "point.hpp"

namespace rj_geometry {

/**
 * A ray, i.e., a half-line starting at an origin and extending infinitely in one direction.
 * Parametrized by an origin point and a unit direction.
 *
 * A zero direction yields a degenerate ray that is just its origin.
 */
class Ray {
public:
    // --- Constructors ---

    Ray() = default;
    /**
     * Initialize a ray with a given origin and direction.
     *
     * @param origin: the point the ray starts from
     * @param direction: the direction the ray extends in; need not be normalized
     */
    Ray(Point origin, Point direction) : origin_{origin}, direction_{direction.normalized()} {}

    // --- Getters ---

    Point origin() const { return origin_; }
    Point direction() const { return direction_; }

    // --- Setters ---

    void set_origin(Point origin) { origin_ = origin; }
    void set_direction(Point direction) { direction_ = direction.normalized(); }

    // --- Operators ---

    bool operator==(const Ray& other) const {
        return origin_ == other.origin_ && direction_ == other.direction_;
    }

    // --- Custom utilities ---

    /**
     * Returns the point on the ray closest to p.
     * Points behind the origin map to the origin.
     */
    Point nearest_point(const Point& p) const;

    /**
     * Returns the shortest distance from p to the ray.
     */
    float dist_to(const Point& p) const;

    std::string to_string() const {
        std::stringstream str;
        str << "Ray<" << origin() << ", " << direction() << ">";
        return str.str();
    }
    friend std::ostream& operator<<(std::ostream& stream, const Ray& ray) {
        return stream << ray.to_string();
    }

private:
    // --- Hidden parameters ---

    Point origin_;
    Point direction_;
};

}  // namespace rj_geometry
