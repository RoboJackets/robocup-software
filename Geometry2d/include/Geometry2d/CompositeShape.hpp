#pragma once

#include "Point.hpp"
#include "Shape.hpp"
#include "Segment.hpp"
#include <vector>
#include <memory>
#include <set>

namespace Geometry2d {

/**
 * A Geometry2d::CompositeShape is a Shape that is made up of other shapes.
 */
class CompositeShape : public Shape {
public:
    CompositeShape(const std::shared_ptr<Shape>& shape) {
        subshapes_.push_back(shape);
    }
    CompositeShape() = default;

    ~CompositeShape() override { clear(); }

    CompositeShape(const CompositeShape& other) {
        for (const auto& itr : other) {
            subshapes_.push_back(std::shared_ptr<Shape>((*itr).clone()));
        }
    }

    [[nodiscard]] Shape* clone() const override;

    [[nodiscard]] bool contains_point(Point pt) const override;
    [[nodiscard]] bool near_point(Point pt, float threshold) const override;

    void add(const std::shared_ptr<Shape>& shape);

    /// adds @comp_shape's subshapes to the receiver
    void add(const CompositeShape& comp_shape);

    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& subshapes() const {
        return subshapes_;
    }

    /// removes all subshapes
    void clear();

    bool empty() { return subshapes_.empty(); }

    [[nodiscard]] unsigned int size() const { return subshapes_.size(); }

    /**
     * Checks if a given shape is in it
     *
     * @param obj The object to collision test
     * @return A bool telling whether or not there were any collisions
     */
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

    // STL typedefs
    using const_iterator = std::vector<std::shared_ptr<Shape>>::const_iterator;
    using iterator = std::vector<std::shared_ptr<Shape>>::iterator;
    using value_type = std::shared_ptr<Shape>;

    // STL Interface
    [[nodiscard]] const_iterator begin() const { return subshapes_.begin(); }
    [[nodiscard]] const_iterator end() const { return subshapes_.end(); }

    iterator begin() { return subshapes_.begin(); }
    iterator end() { return subshapes_.end(); }

    std::string to_string() override {
        std::stringstream str;
        str << "CompositeShape<";
        for (auto& subshape : subshapes_) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

    std::shared_ptr<Shape> operator[](unsigned int index) {
        return subshapes_[index];
    }

    std::shared_ptr<const Shape> operator[](unsigned int index) const {
        return subshapes_[index];
    }

private:
    std::vector<std::shared_ptr<Shape>> subshapes_;
};

}  // namespace Geometry2d
