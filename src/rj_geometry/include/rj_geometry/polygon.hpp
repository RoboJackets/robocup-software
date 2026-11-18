#pragma once

#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rj_geometry_msgs/msg/detail/polygon__struct.hpp>
#include <rj_geometry_msgs/msg/polygon.hpp>

#include "point.hpp"
#include "shape.hpp"

namespace rj_geometry {
class Segment;

/**
 * A polygon.
 * Parametrized by an ordered list of vertices.
 * The boundary runs through the vertices in order, closing from the last back to the first.
 */
class Polygon : public Shape {
public:
    using Msg = rj_geometry_msgs::msg::Polygon;

    // --- Constructors ---

    /** Empty (vertexless) polygon constructor */
    Polygon() = default;
    /**
     * Initialize a polygon from an ordered list of vertices.
     *
     * @param vertices: vertices in boundary order
     *
     * NOTE: no assertions will be raised for degenerate polygons
     */
    Polygon(std::vector<Point> vertices) : vertices_(std::move(vertices)) {}

    // --- Getters ---

    const std::vector<Point>& vertices() const { return vertices_; }

    // --- Setters ---

    void add_vertex(Point pt) { vertices_.push_back(pt); }

    // --- Operators ---

    // ... none for now ...

    // --- Shape virtuals ---

    bool contains_point(const Point& p) const override;
    bool intersects_segment(const Segment& seg) const override;
    bool near_point(const Point& p, float clearance) const override;
    bool near_segment(const Segment& seg, float clearance) const override;
    std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        str << "Polygon<";
        for (size_t i = 0; i < vertices_.size(); ++i) {
            if (i > 0) {
                str << ", ";
            }
            str << vertices_[i];
        }
        str << ">";
        return str.str();
    }

    // --- Shape pseudovirtual clone ---

    std::unique_ptr<Polygon> clone() const;

    // --- Custom utilities ---

    // ... none for now ...

protected:
    // --- Hidden parameters ---

    std::vector<Point> vertices_;

    // --- Shape virtuals ---

    Polygon* clone_impl() const override;

    // --- Custom utilities ---

    // ... none for now ...

};
}  // namespace rj_geometry
