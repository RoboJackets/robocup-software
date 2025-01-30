#pragma once

#include <memory>
#include <set>
#include <sstream>
#include <vector>

#include <rj_geometry_msgs/msg/shape_set.hpp>

#include "shape.hpp"

namespace rj_geometry {

/// This class maintains a collection of Shape objects.
class ShapeSet {
public:
    using Msg = rj_geometry_msgs::msg::ShapeSet;

    ShapeSet() = default;

    /// Initializes the set by iterating from @first to @last, which are
    /// iterators into a collection of std::shared_ptr<Shape>.
    template <class InputIt>
    ShapeSet(InputIt first, InputIt last) {
        while (first != last) {
            add(*first++);
        }
    }

    std::vector<std::shared_ptr<Shape>>& shapes() { return shapes_; }
    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& shapes() const {
        return shapes_;
    }

    void add(std::shared_ptr<Shape> shape) {
        assert(shape != nullptr);
        shapes_.push_back(shape);
    }

    void add(const ShapeSet& other) {
        for (auto shape : other.shapes()) {
            add(shape);
        }
    }

    /// Remove all shapes
    void clear() { shapes_.clear(); }

    /**
     * Get a set of which shapes "hit" the given object.
     *
     * @param obj The object to collision test
     * @return A set of all shapes that collide with the given object
     */
    template <typename T>
    std::set<std::shared_ptr<Shape>> hit_set(const T& obj) const {
        std::set<std::shared_ptr<Shape>> hits;
        for (const auto& shape : shapes_) {
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
        for (const auto& shape : shapes_) {
            if (shape->hit(obj)) {
                return true;
            }
        }
        return false;
    }

    friend std::ostream& operator<<(std::ostream& out,
                                    const ShapeSet& shape_set) {
        out << "ShapeSet: {";
        for (const auto& shape : shape_set.shapes()) {
            out << shape->to_string() << ", ";
        }
        out << "}";
        return out;
    }

private:
    std::vector<std::shared_ptr<Shape>> shapes_;
};

}  // namespace rj_geometry
