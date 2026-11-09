#pragma once

#include "point.hpp"
#include "shape.hpp"
#include "segment.hpp"
#include <vector>
#include <memory>
#include <set>

namespace rj_geometry {

/**
 * A composite shape.
 * Parametrized by a list of subshapes.
 */
class CompositeShape : public Shape {
public:
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
     * Initialize a composite shape as a copy of another.
     *
     * @param other: the composite shape to copy
     *
     * @note each subshape is cloned
     */
    CompositeShape(const CompositeShape& other) {
        for (const auto& itr : other) {
            subshapes_.push_back(std::shared_ptr<Shape>((*itr).clone()));
        }
    }

    /**
     * Destroy this composite shape and clear its subshapes.
     */
    ~CompositeShape() override { clear(); }

    /** Getters */
    [[nodiscard]] const std::vector<std::shared_ptr<Shape>>& subshapes() const {
        return subshapes_;
    }
    bool empty() { return subshapes_.empty(); }
    [[nodiscard]] unsigned int size() const { return subshapes_.size(); }

    /** Setters */

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
     */
    void add(const CompositeShape& comp_shape);

    /**
     * Remove all subshapes.
     */
    void clear();

    /** Overriding virtuals from Shape */
    [[nodiscard]] bool contains_point(const Point& pt) const override;
    [[nodiscard]] bool near_point(const Point& pt, float threshold) const override;
    [[nodiscard]] bool intersects_segment(const Segment& seg) const override;
    [[nodiscard]] bool near_segment(const Segment& seg, float clearance) const override;
    [[nodiscard]] std::vector<Point> get_segment_intersections(const Segment& seg) const override;
    std::string to_string() const override {
        std::stringstream str;
        str << "CompositeShape<";
        for (auto& subshape : subshapes_) {
            str << subshape->to_string() << ", ";
        }
        str << ">";

        return str.str();
    }

    /** Overriding clone utilites from Shape */
    [[nodiscard]] std::unique_ptr<CompositeShape> clone() const;

    /** Custom utilities not from Shape */

    /** STL typedefs */
    using const_iterator = std::vector<std::shared_ptr<Shape>>::const_iterator;
    using iterator = std::vector<std::shared_ptr<Shape>>::iterator;
    using value_type = std::shared_ptr<Shape>;

    /** STL interface */
    [[nodiscard]] const_iterator begin() const { return subshapes_.begin(); }
    [[nodiscard]] const_iterator end() const { return subshapes_.end(); }

    iterator begin() { return subshapes_.begin(); }
    iterator end() { return subshapes_.end(); }

    std::shared_ptr<Shape> operator[](unsigned int index) {
        return subshapes_[index];
    }

    std::shared_ptr<const Shape> operator[](unsigned int index) const {
        return subshapes_[index];
    }

private:
    /** Overriding clone utilites from Shape */
    CompositeShape* clone_impl() const override;

    /** Hidden parameters */
    std::vector<std::shared_ptr<Shape>> subshapes_;
};

}  // namespace rj_geometry
