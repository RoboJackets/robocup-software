
#pragma once

#include "Rect.hpp"
#include "Point.hpp"
#include "Line.hpp"
#include "Circle.hpp"

#include <rj_geometry_msgs/msg/segment.hpp>
#include <memory>

namespace Geometry2d {
using SegmentMsg = rj_geometry_msgs::msg::Segment;

class Segment {
public:
    /** the Segement consists of two points */
    Point pt[2];

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
    float distTo(const Point& other) const;

    /* Returns the relative vector */
    Point delta() const { return pt[1] - pt[0]; }

    /* returns the length of the segment */
    float length() const { return (pt[1] - pt[0]).mag(); }

    bool nearPoint(const Point& point, float threshold) const;
    bool nearSegment(const Segment& other, float threshold) const;

    /** find the nearest point on the segment given @a p */
    Point nearestPoint(const Point& p) const;

    Point nearestPoint(const Line& l) const;

    bool intersects(const Segment& other, Point* intr = nullptr) const;
    bool intersects(const Circle& circle) const;
    bool intersects(const Line& line, Point* intr = 0) const;

    std::string toString() const {
        std::stringstream str;
        str << "Segment<" << pt[0] << ", " << pt[1] << ">";
        return str.str();
    }

    [[nodiscard]] SegmentMsg toROS() const {
        SegmentMsg msg{};
        msg.pt = {pt[0].toROS(), pt[1].toROS()};
        return msg;
    }

    static Segment fromROS(const SegmentMsg& msg) {
        return Segment{Point::fromROS(msg.pt[0]), Point::fromROS(msg.pt[1])};
    }

    friend std::ostream& operator<<(std::ostream& stream, const Segment& seg) {
        return stream << seg.toString();
    }

    bool operator==(const Segment& other) const {
        return pt[0] == other.pt[0] && pt[1] == other.pt[1];
    }
};
}
