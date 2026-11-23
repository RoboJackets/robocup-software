#pragma once

#include <memory>
#include <sstream>
#include <string>

#include "composite_shape.hpp"
#include "point.hpp"
#include "shape_set.hpp"

namespace rj_geometry {

/**
 * A "stadium" shape, made up of two semi-circles about a rectangle (stored as a Polygon,
 * so the stadium may be at any angle)
 * It represents the shape of a track from track and field.
 * Note that this class mostly inherits all functionality from CompositeShape.
 */
class StadiumShape : public CompositeShape {
public:
    // --- Constructors ---

    StadiumShape() = default;

    StadiumShape(Point c1, Point c2, float r);

    // --- Getters ---

    // ... none for now ...

    // --- Setters ---

    /**
     * This enlarges the stadium outward in all directions,
     * moving the edges by the specified amount.
     *
     * Note that this is L2 padding.
     * The result will still be a StadiumShape, with larger "radius".
     *
     * NOTE: This WILL have bad behavior if you have reordered the shapes.
     */
    void pad(float padding);

    // --- Operators ---

    // ... none for now ...

    // --- Shape virtuals ---

    std::string to_string() const override {
        std::stringstream str;
        str << "StadiumShape<";
        for (auto& subshape : subshapes_) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

    // --- Shape pseudovirtual clone ---

    std::unique_ptr<StadiumShape> clone() const;

    // --- Custom utilities ---

    // ... none for now ...
    // DEPRECATE!!! CompositeShape has a getter "subshapes"
    const rj_geometry::ShapeSet drawshapes() const { return ShapeSet(subshapes_); }

protected:
    // --- Hidden parameters ---

    // ... none for now ...

    // --- Shape virtuals ---

    StadiumShape* clone_impl() const override;

    // --- Custom utilities ---

    // ... none for now ...

};

}  // namespace rj_geometry
