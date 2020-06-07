#pragma once

#include <geometry2d/point.h>
#include <geometry2d/segment.h>
#include <geometry2d/shape.h>

#include <memory>
#include <set>
#include <vector>

namespace geometry2d {

/**
 * A geometry2d::CompositeShape is a Shape that is made up of other shapes.
 */
class CompositeShape : public Shape {
public:
    CompositeShape(const std::shared_ptr<Shape>& shape) {
        _subshapes.push_back(shape);
    }
    CompositeShape() = default;

    ~CompositeShape() override { clear(); }

    CompositeShape(const CompositeShape& other) {
        for (const auto& itr : other) {
            _subshapes.push_back(std::shared_ptr<Shape>((*itr).clone()));
        }
    }

    [[nodiscard]] Shape* clone() const override;

    [[nodiscard]] bool containsPoint(Point pt) const override;
    [[nodiscard]] bool nearPoint(Point pt, float threshold) const override;

    void add(const std::shared_ptr<Shape>& shape);

    /// adds @compShape's subshapes to the receiver
    void add(const CompositeShape& compShape);

    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& subshapes() const {
        return _subshapes;
    }

    /// removes all subshapes
    void clear();

    bool empty() { return _subshapes.empty(); }

    [[nodiscard]] unsigned int size() const { return _subshapes.size(); }

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
    [[nodiscard]] const_iterator begin() const { return _subshapes.begin(); }
    [[nodiscard]] const_iterator end() const { return _subshapes.end(); }

    iterator begin() { return _subshapes.begin(); }
    iterator end() { return _subshapes.end(); }

    std::string toString() override {
        std::stringstream str;
        str << "CompositeShape<";
        for (auto& _subshape : _subshapes) {
            str << _subshape->toString() << ", ";
        }
        str << ">";

        return str.str();
    }

    std::shared_ptr<Shape> operator[](unsigned int index) {
        return _subshapes[index];
    }

    std::shared_ptr<const Shape> operator[](unsigned int index) const {
        return _subshapes[index];
    }

private:
    std::vector<std::shared_ptr<Shape>> _subshapes;
};

}  // namespace geometry2d
