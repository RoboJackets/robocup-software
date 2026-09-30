#pragma once

#include <cassert>

#include "line.hpp"
#include "point.hpp"
#include "segment.hpp"

namespace rj_geometry {

/**
 * An arc, i.e., a subtended segment of a circle.
 * Parametrized by a center point, radius, and starting and ending angles.
 */
class Arc {
public:
    /**
     * Initialize an arc with a given center, radius, and starting and ending
     * angle (in radians).
     *
     * @param center: the center point of the underlying circle
     * @param radius: the radius of the underlying circle in the range [0, infty)
     * @param start: the starting angle in radians in the range [-M_PI, M_PI]
     * @param end: the ending angle in radians in the range [-M_PI, M_PI]
     *
     * @note angle 0 radians is aligned with the x axis
     * @note if start > end, an assertion will be thrown
     * @note if radius < 0, an assertion will be thrown
     * @note if start or end is out of range, an assertion will be thrown
     */
    Arc(Point center, float radius, float start, float end) {
        assert(radius >= 0);
        assert(start <= end);
        center_ = center;
        radius_ = radius;
        start_angle_ = start;
        end_angle_ = end;
    }

    /** Getters */
    Point center() const { return center_; }
    float radius() const { return radius_; }
    float start() const { return start_angle_; }
    float end() const { return end_angle_; }

    /** Setters */
    void set_center(Point center) {
        center_ = center;
    }
    void set_radius(float radius) {
        assert(radius >= 0);
        radius_ = radius;
    }
    void set_start(float start) {
        assert(start <= end_angle_);
        start_angle_ = start;
    }
    void set_end(float end) {
        assert(start_angle_ <= end);
        end_angle_ = end;
    }


    /** Geometry */
    std::vector<Point> intersects(const Line& line) const;
    std::vector<Point> intersects(const Segment& segment) const;

    /** DEPRECATE */
    float radius_sq() const { return radius_ * radius_; }

private:
    Point center_;
    float radius_;
    float start_angle_;
    float end_angle_;
};

}  // namespace rj_geometry
