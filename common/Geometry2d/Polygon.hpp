#pragma once

#include <vector>

#include "Rect.hpp"
#include "Segment.hpp"

namespace Geometry2d
{
    class Polygon
    {
    public:
        Polygon() {}
        Polygon(const Rect &rect);
        
        /// Creates a rectangle of arbitrary orientation which encloses
        /// all points within a distance r of the given segment.
        Polygon(const Segment &seg, float r)
        {
            init(seg, r, seg.length());
        }
        
        /// Same as above but with length given to avoid a square root.
        Polygon(const Segment &seg, float r, float length)
        {
            init(seg, r, length);
        }
        
        bool contains(const Point &pt) const;
        bool intersects(const Rect &rect) const;
        bool intersects(const Polygon &other) const;
        
        /// Returns true if this polygon contains any vertex of other.
        bool containsVertex(const Polygon &other) const;
        
        bool nearPoint(const Point &pt, float threshold) const;
        bool nearSegment(const Segment &seg, float threshold) const;
        
        Rect bbox() const;
        
        std::vector<Point> vertices;
    
    protected:
        /// Used by constructors
        void init(const Segment &seg, float r, float length);
    };
}
