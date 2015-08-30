#pragma once

#include <vector>

#include "Shape.hpp"
#include "Rect.hpp"
#include "Segment.hpp"

namespace Geometry2d {
class Polygon : public Shape {
public:
    Polygon() {}
    Polygon(const Rect& rect);
    Polygon(std::vector<Point> verts);

    /// Creates a rectangle of arbitrary orientation which encloses / all points
    // within a distance r of the given segment.
    Polygon(const Segment& seg, float r) { init(seg, r, seg.length()); }

    /// Same as above but with length given to avoid a square root.
    Polygon(const Segment& seg, float r, float length) { init(seg, r, length); }

    Polygon(const Polygon& other) : vertices(other.vertices) {}

    Shape* clone() const override;

    bool containsPoint(const Point& pt) const override;

    bool intersects(const Rect& rect) const;
    bool intersects(const Polygon& other) const;

    bool hit(const Geometry2d::Point& pt) const override;
    bool hit(const Geometry2d::Segment& seg) const override;

    /// Returns true if this polygon contains any vertex of other.
    bool containsVertex(const Polygon& other) const;

    bool nearPoint(const Point& pt, float threshold) const;
    bool nearSegment(const Segment& seg, float threshold) const;

    Rect bbox() const;

    std::vector<Point> vertices;

    void addVertex(const Point& pt) { vertices.push_back(pt); }

    std::string toString() override {
        std::stringstream str;
        str << "Polygon<";
        for (int i = 0; i < vertices.size(); i++) str << vertices[i] << ", ";
        str << ">";
        return str.str();
    }

protected:
    /// Used by constructors
    void init(const Segment& seg, float r, float length);
};
}
