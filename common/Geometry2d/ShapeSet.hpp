#pragma once

#include "Shape.hpp"

#include <memory>
#include <set>
#include <vector>

namespace Geometry2d {

/// This class maintains a collection of Shape objects.
class ShapeSet {
public:
    ShapeSet() {}

    /// Initializes the set by iterating from @first to @last, which are
    /// iterators into a collection of std::shared_ptr<Shape>.
    template <class InputIt>
    ShapeSet(InputIt first, InputIt last) {
        while (first != last) {
            add(*first++);
        }
    }

    std::vector<std::shared_ptr<Shape>> shapes() { return _shapes; }
    const std::vector<std::shared_ptr<Shape>> shapes() const { return _shapes; }

    void add(std::shared_ptr<Shape> shape) {
        if (!shape)
            throw std::runtime_error("Attempt to add null shape to ShapeSet");
        _shapes.push_back(shape);
    }

    void add(const ShapeSet& other) {
        for (auto shape : other.shapes()) {
            add(shape);
        }
    }

    /// Remove all shapes
    void clear() { _shapes.clear(); }

    /**
     * Get a set of which shapes "hit" the given object.
     *
     * @param obj The object to collision test
     * @return A set of all shapes that collide with the given object
     */
    template <typename T>
    std::set<std::shared_ptr<Shape>> hitSet(const T& obj) const {
        std::set<std::shared_ptr<Shape>> hits;
        for (const auto& shape : _shapes) {
            if (shape->hit(obj)) {
                hits.insert(shape);
            }
        }
        return hits;
    }

    /**
     * Check if any of the shapes in this set "hit" the given object.
     *
     * @param obj The object to collision test
     * @return True if one of the contained shapes hits the object.
     */
    template <typename T>
    bool hit(const T& obj) const {
        return !hitSet<T>(obj).empty();
    }

private:
    std::vector<std::shared_ptr<Shape>> _shapes;
};

}  // namespace Geometry2d
