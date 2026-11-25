#pragma once

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "point.hpp"
#include "segment.hpp"
#include "shape.hpp"

namespace rj_geometry {

/**
 * A composite shape.
 * Parametrized by a list of subshapes.
 */
class CompositeShape : public Shape {
public:
    // --- Constructors ---

    /**
     * Initialize an empty composite shape.
     */
    CompositeShape() = default;
    /**
     * Initialize a composite shape containing a single subshape.
     *
     * @param shape: the subshape to store
     */
    CompositeShape(const std::shared_ptr<Shape>& shape) {
        subshapes_.push_back(shape);
    }

    /**
     * Destroy this composite shape and clear its subshapes.
     */
    ~CompositeShape() override { clear(); }

    // --- Getters ---

    const std::vector<std::shared_ptr<Shape>>& subshapes() const { return subshapes_; }
    bool empty() { return subshapes_.empty(); }
    unsigned int size() const { return subshapes_.size(); }

    // --- Setters ---

    /**
     * Add a subshape.
     *
     * @param shape: the subshape to store
     *
     * @note if shape is null, it is ignored
     */
    void add(const std::shared_ptr<Shape>& shape);
    /**
     * Add comp_shape's subshapes to this composite shape.
     *
     * @param comp_shape: the composite shape whose subshapes are added
     *
     * @note subshape pointers are shared, not cloned; null subshapes are ignored
     */
    void add(const CompositeShape& comp_shape);
    /**
     * Remove all subshapes.
     */
    void clear();

    // --- Operators ---

    std::shared_ptr<Shape> operator[](unsigned int index) {
        return subshapes_[index];
    }
    std::shared_ptr<const Shape> operator[](unsigned int index) const {
        return subshapes_[index];
    }

    // --- Shape virtuals ---

    bool contains_point(const Point& pt) const override;
    bool near_point(const Point& pt, float threshold) const override;
    bool intersects_segment(const Segment& seg) const override;
    bool near_segment(const Segment& seg, float clearance) const override;
    std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        str << "CompositeShape<";
        for (auto& subshape : subshapes_) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

    // --- Shape pseudovirtual clone ---

    std::unique_ptr<CompositeShape> clone() const;

    // --- Custom utilities ---

    // artificially implementing the STL interface
    using const_iterator = std::vector<std::shared_ptr<Shape>>::const_iterator;
    using iterator = std::vector<std::shared_ptr<Shape>>::iterator;
    using value_type = std::shared_ptr<Shape>;

    [[nodiscard]] const_iterator begin() const { return subshapes_.begin(); }
    [[nodiscard]] const_iterator end() const { return subshapes_.end(); }

    iterator begin() { return subshapes_.begin(); }
    iterator end() { return subshapes_.end(); }

protected:
    // --- Hidden parameters ---

    std::vector<std::shared_ptr<Shape>> subshapes_;

    // --- Shape virtuals

    CompositeShape* clone_impl() const override;

    CompositeShape(const CompositeShape& other);
    CompositeShape(CompositeShape&& other) = default;
    CompositeShape& operator=(const CompositeShape& other);
    CompositeShape& operator=(CompositeShape&& other) = default;

    // --- Custom utilities ---

    // ... none for now ...

};

}  // namespace rj_geometry
