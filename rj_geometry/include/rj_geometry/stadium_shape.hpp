#pragma once

#include "point.hpp"
#include "shape.hpp"
#include "segment.hpp"
#include "polygon.hpp"
#include "shape_set.hpp"
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

    StadiumShape(const StadiumShape& other) {
        for (const auto& shape : other.subshapes_) {
            std::shared_ptr<Shape> itr_shape = std::shared_ptr<Shape>(shape->clone());
            subshapes_.push_back(itr_shape);
        }
        drawshapes_ = other.drawshapes_;
    }

    [[nodiscard]] Shape* clone() const override;

    [[nodiscard]] bool contains_point(Point pt) const override;
    [[nodiscard]] bool near_point(Point pt, float threshold) const override;

    using const_iterator = std::vector<std::shared_ptr<Shape>>::const_iterator;
    using iterator = std::vector<std::shared_ptr<Shape>>::iterator;

    [[nodiscard]] const_iterator begin() const { return subshapes_.begin(); }
    [[nodiscard]] const_iterator end() const { return subshapes_.end(); }

    iterator begin() { return subshapes_.begin(); }
    iterator end() { return subshapes_.end(); }

    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& subshapes() const {
        return subshapes_;
    }

    [[nodiscard]] const rj_geometry::ShapeSet drawshapes() const {
        return drawshapes_;
    }

    std::shared_ptr<Shape> operator[](unsigned int index) {
        return subshapes_[index];
    }

    std::shared_ptr<const Shape> operator[](unsigned int index) const {
        return subshapes_[index];
    }

     template <typename T>
    [[nodiscard]] bool hit(const T& obj) const {
        for (const auto& it : *this) {
            if (it->hit(obj)) {
                return true;
            }
        }

        return false;
    }

    [[nodiscard]] bool hit(Point pt) const override { return hit<Point>(pt); }

    [[nodiscard]] bool hit(const Segment& seg) const override {
        return hit<Segment>(seg);
    }

     std::string to_string() override {
        std::stringstream str;
        str << "StadiumShape<";
        for (auto& subshape : subshapes_) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

protected:
    void init(Point c1, Point c2, float r);

private:
    std::vector<std::shared_ptr<Shape>> subshapes_;
    rj_geometry::ShapeSet drawshapes_;
};

}