#pragma once

#include <memory>
#include <set>
#include <vector>

#include "composite_shape.hpp"
#include "point.hpp"
#include "polygon.hpp"
#include "segment.hpp"
#include "shape_set.hpp"

namespace rj_geometry {

/**
 * A rj_geometry::StadiumShape is a CompositeShape made up of 2 circles and a polygon.
 * It represents the shape of a track from track and field.
 */
class StadiumShape : public CompositeShape {
public:
    ~StadiumShape() = default;

    StadiumShape() = default;

    StadiumShape(Point c1, Point c2, float r);

    StadiumShape(const StadiumShape& other) : CompositeShape(other) {
        drawshapes_ = other.drawshapes_;
    }

    [[nodiscard]] Shape* clone() const override;

    [[nodiscard]] const rj_geometry::ShapeSet drawshapes() const { return drawshapes_; }

    std::string to_string() override {
        std::stringstream str;
        str << "StadiumShape<";
        for (auto& subshape : subshapes()) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

private:
    rj_geometry::ShapeSet drawshapes_;
};

}  // namespace rj_geometry