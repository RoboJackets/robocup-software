#include "Polygon.hpp"
#include <Constants.hpp>

Geometry2d::Polygon::Polygon(const Geometry2d::Rect &rect)
{
    vertices.resize(4);
    vertices[0] = rect.pt[0];
    vertices[1] = Point(rect.pt[1].x, rect.pt[0].y);
    vertices[2] = rect.pt[1];
    vertices[3] = Point(rect.pt[0].x, rect.pt[1].y);
}

void Geometry2d::Polygon::init(const Geometry2d::Segment &seg, float r, float length)
{
    Geometry2d::Point dir;
    if (length)
    {
        dir = seg.delta() / length;
    } else {
        // Degenerate segment, so direction doesn't matter as long as it's a unit vector.
        dir = Point(1, 0);
    }
    
    Geometry2d::Point v = dir * r;
    Geometry2d::Point u = v.perpCCW();
    
    vertices.resize(4);
    vertices[0] = seg.pt[0] - u - v;
    vertices[1] = seg.pt[0] + u - v;
    vertices[2] = seg.pt[1] + u + v;
    vertices[3] = seg.pt[1] - u + v;
}

Geometry2d::Rect Geometry2d::Polygon::bbox() const
{
    Geometry2d::Rect rect(vertices[0]);
    
    for (unsigned int i = 1; i < vertices.size(); ++i)
    {
        rect.expand(vertices[i]);
    }
    
    return rect;
}

bool Geometry2d::Polygon::intersects(const Geometry2d::Rect &rect) const
{
    return intersects(Geometry2d::Polygon(rect));
}

bool Geometry2d::Polygon::intersects(const Geometry2d::Polygon &other) const
{
    return containsVertex(other) || other.containsVertex(*this);
}

bool Geometry2d::Polygon::containsVertex(const Geometry2d::Polygon &other) const
{
    for (unsigned int i = 0; i < other.vertices.size(); ++i)
    {
        if (contains(other.vertices[i]))
        {
            return true;
        }
    }
    
    return false;
}

bool Geometry2d::Polygon::nearPoint(const Geometry2d::Point &pt, float threshold) const
{
    if (contains(pt))
    {
        return true;
    }
    
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); ++j)
    {
        Geometry2d::Segment edge(vertices[i], vertices[j]);
        if (edge.nearPoint(pt, threshold))
        {
            return true;
        }
        
        i = j;
    }
    
    return false;
}

bool Geometry2d::Polygon::nearSegment(const Geometry2d::Segment &seg, float threshold) const
{
    if (contains(seg.pt[0]) || contains(seg.pt[1]))
    {
        return true;
    }
    
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); ++j)
    {
        Geometry2d::Segment edge(vertices[i], vertices[j]);
        if (edge.nearSegment(seg, threshold))
        {
            return true;
        }
        
        i = j;
    }
    
    return false;
}

bool Geometry2d::Polygon::contains(const Geometry2d::Point &pt) const
{
    //FIXME (Ben) - Replace this with the optimized wrap-number test.
    
    // http://www.geometryalgorithms.com/Archive/algorithm_0103/algorithm_0103.htm
    // isLeft in their description does not actually indicate if a point is on the -X
    // side of a line.  It is the same as my Line::point_side.
    //
    // Simple explanation:
    // Consider a ray along +X starting at this point.
    // Go around the polygon and look at every intersection of an edge with this ray.
    // Since you are following the polygon's edges in order, if this point is outside
    // the polygon the ray will intersect the same number of up-going and down-going edges.
    //
    // Consider the case of a convex polygon with this point outside:
    // The ray will intersect exactly one up-going edge and one down-going edge.
    // For a non-convex polygon, the ray will intersect zero or more additional pairs
    // of up-going and down-going edges.
    //
    // Increment count for each up-going edge and decrement for each down-going edge.
    // count==0 iff the point is outside.
    //
    // This handles cases where the ray intersects vertices too.
    // Only the endpoint with the smallest Y-value is considered.
    // If the ray is outside the polygon, then both edges will be counted
    // because one will be up-going and one down-going at the vertex.
    // If the ray is inside the polygon, both edges are going in the same direction
    // so the vertex will only be checked for one of them.

    int count = 0;
    unsigned int i = vertices.size() - 1;
    for (unsigned int j = 0; j < vertices.size(); j++)
    {
        const Point &p1 = vertices[i];
        const Point &p2 = vertices[j];
        i = j;
        
        if (p1.y <= pt.y)
        {
            if (p2.y > pt.y)
            {
                // Edge is going up
                if (Line(p1, p2).pointSide(pt) > 0)
                {
                    ++count;
                }
            }
        } else {
            if (p2.y <= pt.y)
            {
                // Edge is going down
                if (Line(p1, p2).pointSide(pt) < 0)
                {
                    --count;
                }
            }
        }
    }
    
    return count != 0;
}

bool Geometry2d::Polygon::hit(const Geometry2d::Point &pt) const
{
    return nearPoint(pt, Robot_Radius);
}

bool Geometry2d::Polygon::hit(const Geometry2d::Segment &seg) const
{
    return nearSegment(seg, Robot_Radius);
}
