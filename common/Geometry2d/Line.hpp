#pragma once

#include "Point.hpp"
#include "TransformMatrix.hpp"

namespace Geometry2d {
class Circle;
class Segment;

class Line {
public:
    Line() {}

    explicit Line(Point p1, Point p2) : pt{p1, p2} {
        // assert(p1 != p2);
    }

    explicit Line(const Line& other) : Line(other.pt[0], other.pt[1]) {}

    explicit Line(const Segment&);

    Point delta() const { return pt[1] - pt[0]; }

    /**
    returns the shortest distance between the line and a point
    @param other the point to find the distance to
    @return the distance to the point from the line
    */
    float distTo(Point other) const;

    /**
    Applies a transformation matrix to the line.
    @param t the transformation matrix to perform on the line
    */
    /*
    void transform(const TransformMatrix& t) {
        pt[0] = t * pt[0];
        pt[1] = t * pt[1];
    }
     */

    /**
    Test for line intersections.
    If the two lines intersect, then the function returns true, else false.
    Also, if the lines intersect, then intr is set to the intersection point.
    @param other the line to test for intersection with
    @param intr set to the intersection point if the lines intersect
    @return true if the lines intersect, false otherwise
    */
    bool intersects(const Line& other, Point* intersection = nullptr) const;

    bool intersects(const Segment& other, Point* intersection = nullptr) const;

    static bool intersects(const Line& line1, const Line& line2,
                           Point* intersection = nullptr);

    /** returns the points of intersection b/t circle and line */
    bool intersects(const Circle& circle, Point* p1 = nullptr,
                    Point* p2 = nullptr) const;

    /**
     * tells you which side of the line you are on
     * @return the sine of the non-zero portion of the cross product
     */
    // TODO check this
    float pointSide(const Point& p) const {
        Point d = delta();
        Point v = p - pt[0];

        return (d.x * v.y - v.x * d.y);
    }

    /**
     * Returns the point on the line closest to p.
     */
    Point nearestPoint(Point p) const;

    /** the line consists of two points */
    Point pt[2];

    std::string toString() {
        std::stringstream str;
        str << "Line<" << pt[0] << ", " << pt[1] << ">";
        return str.str();
    }
};
}
