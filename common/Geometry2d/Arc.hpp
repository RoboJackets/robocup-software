//
// Created by matt on 7/19/15.
//

#pragma once

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
        _radius = -1;
        _start_angle = 0;
        _end_angle = 0;
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
        _center = center;
        _radius = radius;
        _start_angle = start;
        _end_angle = end;
    }

    Point center() const { return _center; }

    void setCenter(Point center) { _center = center; }

    float radius() const { return _radius; }

    void setRadius(float radius) { _radius = radius; }

    float start() const { return _start_angle; }

    void setStart(float start) { _start_angle = start; }

    float end() const { return _end_angle; }

    void setEnd(float end) { _end_angle = end; }

    float radius_sq() const { return _radius * _radius; }

    std::vector<Point> intersects(const Line& line) const;

    std::vector<Point> intersects(const Segment& segment) const;

private:
    Point _center;
    float _radius;
    float _start_angle;
    float _end_angle;
};

}  // namespace Geometry2d
