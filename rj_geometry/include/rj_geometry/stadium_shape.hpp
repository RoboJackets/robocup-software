#pragma once

#include "point.hpp"
#include "shape.hpp"
#include "segment.hpp"
#include "polygon.hpp"
#include <vector>
#include <memory>
#include <set>

namespace rj_geometry {

/**
 * A rj_geometry::StadiumShape is a Shape that is made up of 2 circles and a polygon. It represents the shape of a track from track and field.
 */
class StadiumShape : public Shape {
public:
    ~StadiumShape() = default;

    StadiumShape() = default;

    StadiumShape(Point c1, Point c2, float r) {
        init(c1, c2, r);
    }

    [[nodiscard]] Shape* clone() const override;

    [[nodiscard]] bool contains_point(Point pt) const override;
    [[nodiscard]] bool near_point(Point pt, float threshold) const override;

    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& subshapes() const {
        return subshapes_;
    }

protected:
    void init(Point c1, Point c2, float r);

private:
    std::vector<std::shared_ptr<Shape>> subshapes_;
};

}