#pragma once

#include <vector>

#include <rj_geometry_msgs/msg/polygon.hpp>

#include "rect.hpp"
#include "segment.hpp"
#include "shape.hpp"

namespace rj_geometry {

/**
 * A polygon.
 * Parametrized by an ordered list of vertices.
 */
class Polygon : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Polygon;

    /**
     * Initialize an empty polygon.
     */
    Polygon() {}

    /**
     * Initialize a polygon from the corners of an axis-aligned rectangle.
     *
     * @param rect: the rectangle whose corners become the polygon vertices
     */
    Polygon(const Rect& rect);

    /**
     * Initialize a polygon from an ordered list of vertices.
     *
     * @param verts: vertices in boundary order
     */
    Polygon(std::vector<Point> verts);

    /**
     * Initialize a rectangle of arbitrary orientation which encloses all points
     * within a distance r of the given segment.
     *
     * @param seg: the segment to expand
     * @param r: the distance around the segment
     */
    Polygon(const Segment& seg, float r) { init(seg, r, seg.length()); }

    /**
     * Initialize a rectangle of arbitrary orientation which encloses all points
     * within a distance r of the given segment.
     *
     * @param seg: the segment to expand
     * @param r: the distance around the segment
     * @param length: the precomputed length of seg, to avoid a square root
     */
    Polygon(const Segment& seg, float r, float length) { init(seg, r, length); }

    /**
     * Initialize a polygon as a copy of another polygon.
     *
     * @param other: the polygon to copy
     */
    Polygon(const Polygon& other) : vertices(other.vertices) {}

    /** Getters */
    Rect bbox() const;

    /** Setters */
    void add_vertex(Point pt) { vertices.push_back(pt); }

    /** Overriding virtuals from Shape */
    bool contains_point(const Point& pt) const override;
    bool near_point(const Point& pt, float threshold) const override;
    bool intersects_segment(const Segment& seg) const override;
    bool near_segment(const Segment& seg, float threshold) const override;
    [[nodiscard]] std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        str << "Polygon<";
        for (auto vertex : vertices) str << vertex << ", ";
        str << ">";
        return str.str();
    }

    /** Overriding clone utilites from Shape */
    [[nodiscard]] std::unique_ptr<Polygon> clone() const;

    /** Custom utilities not from Shape */
    bool intersects(const Rect& rect) const;
    bool intersects(const Polygon& other) const;

    /**
     * Returns true if this polygon contains any vertex of other.
     *
     * @param other: the polygon whose vertices are tested
     */
    bool contains_vertex(const Polygon& other) const;

    /** Parameters */
    std::vector<Point> vertices;

protected:
    /** Overriding clone utilites from Shape */
    Polygon* clone_impl() const override;

    /**
     * Used by the segment constructors.
     *
     * @param seg: the segment to expand
     * @param r: the distance around the segment
     * @param length: the length of seg
     */
    void init(const Segment& seg, float r, float length);
};
}
