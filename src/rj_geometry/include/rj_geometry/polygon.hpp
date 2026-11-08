#pragma once

#include <vector>

#include <rj_geometry_msgs/msg/polygon.hpp>

#include "rect.hpp"
#include "segment.hpp"
#include "shape.hpp"

namespace rj_geometry {

class Polygon : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Polygon;

    Polygon() {}
    Polygon(const Rect& rect);
    Polygon(std::vector<Point> verts);

    /// Creates a rectangle of arbitrary orientation which encloses / all points
    // within a distance r of the given segment.
    Polygon(const Segment& seg, float r) { init(seg, r, seg.length()); }

    /// Same as above but with length given to avoid a square root.
    Polygon(const Segment& seg, float r, float length) { init(seg, r, length); }

    Polygon(const Polygon& other) : vertices(other.vertices) {}

    [[nodiscard]] std::unique_ptr<Polygon> clone() const;

    bool contains_point(const Point& pt) const override;

    bool intersects(const Rect& rect) const;
    bool intersects(const Polygon& other) const;
    bool intersects_segment(const Segment& seg) const override;

    [[nodiscard]] std::vector<Point> get_segment_intersections(const Segment& seg) const override;

    /// Returns true if this polygon contains any vertex of other.
    bool contains_vertex(const Polygon& other) const;

    bool near_point(const Point& pt, float threshold) const override;
    bool near_segment(const Segment& seg, float threshold) const override;

    Rect bbox() const;

    std::vector<Point> vertices;

    void add_vertex(Point pt) { vertices.push_back(pt); }

    std::string to_string() const override {
        std::stringstream str;
        str << "Polygon<";
        for (auto vertex : vertices) str << vertex << ", ";
        str << ">";
        return str.str();
    }

protected:
    /// Used by constructors
    void init(const Segment& seg, float r, float length);

    Polygon* clone_impl() const override;
};
}
