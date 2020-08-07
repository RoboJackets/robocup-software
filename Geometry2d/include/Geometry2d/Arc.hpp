//
// Created by matt on 7/19/15.
//

#pragma once

// TODO(1483): Sort and group / style includes with clang-format
#include "Line.hpp"
#include "Point.hpp"
#include "Segment.hpp"

namespace Geometry2d {

/**
 * An arc (a segment of a circle) specified by a center point, a radius and a
 * starting and ending angle.
 *
 * @note: the starting and ending angles should be in the range [-M_PI, M_PI]
 *      or intersection functions will not work as expected. In addition,
 *      start should be less than or equal to end
 */
class Arc {
public:
    /**
     * Default-initialze an arc to the empty arc.
     */
    Arc() {
        radius_ = -1;
        start_angle_ = 0;
        end_angle_ = 0;
    }

    /**
     * Initialize an arc with a given center, radius, and starting and ending
     * angle (in radians).
     *
     * @param center: the center point of the characteristic circle
     * @param radius: the radius of the characteristic circle
     * @param start: the starting angle in radians in the range [-M_PI, M_PI]
     * @param end: the ending angle in radians in the range [-M_PI, M_PI]
     *
     * @note angle 0 radians is aligned with the x axis.
     */
    Arc(Point center, float radius, float start, float end) {
        center_ = center;
        radius_ = radius;
        start_angle_ = start;
        end_angle_ = end;
    }

    Point center() const { return center_; }

    void set_center(Point center) { center_ = center; }

    float radius() const { return radius_; }

    void set_radius(float radius) { radius_ = radius; }

    float start() const { return start_angle_; }

    void set_start(float start) { start_angle_ = start; }

    float end() const { return end_angle_; }

    void set_end(float end) { end_angle_ = end; }

    float radius_sq() const { return radius_ * radius_; }

    std::vector<Point> intersects(const Line& line) const;

    std::vector<Point> intersects(const Segment& segment) const;

private:
    Point center_;
    float radius_;
    float start_angle_;
    float end_angle_;
};

}  // namespace Geometry2d
