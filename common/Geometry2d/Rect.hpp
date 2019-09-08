#pragma once

#include "Shape.hpp"
#include "Point.hpp"

namespace Geometry2d {
class Segment;

/// Represents a rectangle by storing two opposite corners.  They may be upper-
/// left and lower-right or any other pair of diagonal corners.
class Rect : public Shape {
private:
    int CohenSutherlandOutCode(const Point& other) const;
public:
    Rect() {}

    Rect(Point p1) { pt[0] = pt[1] = p1; }

    Rect(Point p1, Point p2) {
        pt[0] = p1;
        pt[1] = p2;
    }

    Rect(const Rect& other) {
        pt[0] = other.pt[0];
        pt[1] = other.pt[1];
    }

    Shape* clone() const override;

    Rect& operator+=(Point offset) {
        pt[0] += offset;
        pt[1] += offset;

        return *this;
    }

    Rect& operator-=(Point offset) {
        pt[0] -= offset;
        pt[1] -= offset;

        return *this;
    }

    Rect operator+(Point offset) {
        return Rect(pt[0] + offset, pt[1] + offset);
    }

    Rect operator*(float s) { return Rect(pt[0] * s, pt[1] * s); }

    Rect& operator*=(float s) {
        pt[0] *= s;
        pt[1] *= s;

        return *this;
    }

    bool containsRect(const Rect& other) const;

    bool containsPoint(Point other) const override;

    bool hit(Point pt) const override;

    bool hit(const Segment& seg) const override;

    Point center() const { return (pt[0] + pt[1]) / 2; }

    /*
    * The expand function will make the rectangle larger to include
    * the given point (just large enough) 
    * This function will alter the points defining the rect to the bottom left
    * and the top right
    */
    void expand(Point pt);
    
    /*
    * The expand function will make the rectangle larger to include
    * the given rectangle (just large enough)
    * This function will alter the points defining the rect to the bottom left
    * and the top right
    */
    void expand(const Rect& rect);

    /*
    * Makes the rectangle bigger in all direction by the padding amount
    * especially useful around the goalboxes to movement
    * This function will alter the points defining the rect to the bottom left
    * and the top right
    */
    void pad(float padding);

    float minx() const { return std::min(pt[0].x(), pt[1].x()); }
    float miny() const { return std::min(pt[0].y(), pt[1].y()); }
    float maxx() const { return std::max(pt[0].x(), pt[1].x()); }
    float maxy() const { return std::max(pt[0].y(), pt[1].y()); }

    /*
    * The corners() function lists the 4 corners of the rectangle
    * in a predictable order regardless of the 2 corners defined on
    * construction.
    * BottomLeft, TopLeft, TopRight, BottomRight
    * exposed to python as corners()
    */
    std::vector<Point> corners();


    bool nearPoint(Point pt, float threshold) const override;
    bool nearSegment(const Segment& seg, float threshold) const;

    bool intersects(const Rect& other) const;
    
    /*
    * Calculates intersection point(s) between the rectangle and a line segment
    * Uses Cohen Sutherland line clipping algorithm
    */
    std::tuple<bool, std::vector<Point> > intersects(const Segment& other) const;

    /*
    * Calculates the code for the Cohen Sutherland algorithm
    * bit string represents how a point relates to the rect 
    */
    

    Point pt[2];

    std::string toString() override {
        std::stringstream str;
        str << *this;
        return str.str();
    }

    friend std::ostream& operator<<(std::ostream& out, const Rect& rect) {
        return out << "Rect<" << rect.pt[0] << ", " << rect.pt[1] << ">";
    }
};
}
