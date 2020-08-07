
#pragma once

#include <memory>
#include <rj_geometry_msgs/msg/segment.hpp>

#include "Circle.hpp"
#include "Line.hpp"
#include "Point.hpp"
#include "Rect.hpp"

namespace Geometry2d {

class Segment {
public:
    using Msg = rj_geometry_msgs::msg::Segment;

    /** the Segement consists of two points */
    std::array<Point, 2> pt;

    Segment() {}

    Segment(Point p1, Point p2) : pt{p1, p2} {}

    explicit Segment(const Line& other) : Segment(other.pt[0], other.pt[1]) {}

    Segment& operator+=(const Point& delta) {
        pt[0] += delta;
        pt[1] += delta;
        return *this;
    }

    Point center() const { return (pt[0] + pt[1]) / 2; }

    /* returns a bounding box of type Rect */
    Rect bbox() const;

    /* returns the distance to point other */
    float dist_to(const Point& other) const;

    /* Returns the relative vector */
    Point delta() const { return pt[1] - pt[0]; }

    /* returns the length of the segment */
    float length() const { return (pt[1] - pt[0]).mag(); }

    bool near_point(const Point& point, float threshold) const;
    bool near_segment(const Segment& other, float threshold) const;

    /** find the nearest point on the segment given @a p */
    Point nearest_point(const Point& p) const;

    Point nearest_point(const Line& l) const;

    bool intersects(const Segment& other, Point* intr = nullptr) const;
    bool intersects(const Circle& circle) const;
    bool intersects(const Line& line, Point* intr = 0) const;

    std::string to_string() const {
        std::stringstream str;
        str << "Segment<" << pt[0] << ", " << pt[1] << ">";
        return str.str();
    }

    friend std::ostream& operator<<(std::ostream& stream, const Segment& seg) {
        return stream << seg.to_string();
    }

    bool operator==(const Segment& other) const { return pt == other.pt; }
};
}
