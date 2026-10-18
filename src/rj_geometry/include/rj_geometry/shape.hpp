#pragma once

#include <memory>
#include <ostream>
#include <string>

#include "point.hpp"

namespace rj_geometry {

class Segment;

/**
 * The shape class provides the interface to all shapes that are subclasses.
 */
class Shape {
public:
    /**
     * Returns a new pointer to a copy of the dynamic type.
     *
     * I'm not sure if we need covariant return polymorphism in our stack, so this may be overkill.
     * Covariant return allows Shape* to be natively overridden by Circle*, e.g.
     * However, it does not allow unique_ptr<Shape> to be overridden by unique_ptr<Circle>.
     *
     * clone_impl makes a raw pointer, then this converts it to a new pointer.
     */
    [[nodiscard]] std::unique_ptr<Shape> clone() const {
        return std::unique_ptr<Shape>(clone_impl());
    }

    /**
     * Returns True if the given point lies within the shape (boundary-inclusive).
     */
    [[nodiscard]] virtual bool intersects_point(const Point& p) const = 0;

    /**
     * Returns True if the given point is in the shape or
     * within the specified L2 clearance region around the shape.
     *
     * Identical to intersects_point if clearance = 0.
     *
     * Sample usage:
     *  - set p to the robot position
     *  - set clearance to be robot's radius
     *  - returns whether the robot would collide with Shape at that position
     */
    [[nodiscard]] virtual bool near_point(const Point& p, float clearance) const = 0;
    // previously called hit ^ (a note for myself when refactoring, if you see this later, delete)

    /**
     * Returns True if any part of the given segment lies within the shape (boundary-inclusive).
     */
    [[nodiscard]] virtual bool intersects_segment(const Segment& seg) const = 0;

    /**
     * Returns True if the given segment is in the shape or
     * within the specified L2 clearance region around the shape.
     *
     * Identical to intersects_segment if clearance = 0.
     *
     * Sample usage:
     *  - set seg to be a proposed robot path
     *  - set clearance to be robot's radius
     *  - returns whether the robot would collide with Shape anywhere on that path
     */
    [[nodiscard]] virtual bool near_segment(const Segment& seg, float clearance) const = 0;

    /**
     * Converts to a printable string.
     * Currently pure virtual because I would be mad if I got a debug message saying "Shape".
     */
    [[nodiscard]] virtual std::string to_string() const = 0;
    friend std::ostream& operator<<(std::ostream& stream, const Shape& shape) {
        return stream << shape.to_string();
    }

    /**
     * Publicize the destructor, but not the others.
     */
    virtual ~Shape() = default;
protected:
    /**
     * Hiding these to prevent slicing. Use clone() for copies.
     */
    Shape() = default;
    Shape(const Shape& other) = default;
    Shape(Shape&& other) = default;
    Shape& operator=(const Shape& other) = default;
    Shape& operator=(Shape&& other) = default;

    /**
     * A heap-allocated raw pointer copy, only for usage in clone().
     */
    virtual Shape* clone_impl() const = 0;
};

}  // namespace rj_geometry
