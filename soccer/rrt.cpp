#include <stdlib.h>
#include <boost/foreach.hpp>
#include <algorithm>

#include "rrt.hpp"

#include <Constants.hpp>

using namespace std;

Geometry::Point2d RRT::randomPoint()
{
    float x = (Constants::Field::Width - Constants::Robot::Diameter) * (drand48() - 0.5f) + Constants::Robot::Radius;
    float y = (Constants::Field::Length - Constants::Robot::Diameter) * drand48() + Constants::Robot::Radius;
    
    return Geometry::Point2d(x, y);
}

////////

RRT::Obstacle::~Obstacle()
{
}

bool RRT::Obstacle::hit(const Geometry::Segment &seg)
{
    return false;
}

////////

RRT::CircleObstacle::CircleObstacle(Geometry::Point2d center, float radius)
{
    _center = center;
    _radius = radius;
}

bool RRT::CircleObstacle::hit(const Geometry::Segment &seg)
{
    return seg.nearPoint(_center, _radius + Constants::Robot::Radius);
}

////////

RRT::ObstacleSet::~ObstacleSet()
{
    BOOST_FOREACH(Obstacle *obs, _obstacles)
    {
        delete obs;
    }
}

void RRT::ObstacleSet::add(Obstacle *obs)
{
    _obstacles.push_back(obs);
}

bool RRT::ObstacleSet::hit(const Geometry::Segment &seg) const
{
    BOOST_FOREACH(Obstacle *obs, _obstacles)
    {
        if (obs->hit(seg))
        {
            return true;
        }
    }
    
    return false;
}

////////

RRT::Tree::Tree(const ObstacleSet &obstacles, Geometry::Point2d pt):
    _obstacles(obstacles)
{
    step = 0.1f;

    start = new Point(pt);
    points.push_back(start);
}

RRT::Tree::~Tree()
{
    // Delete all points
    BOOST_FOREACH(Point *pt, points)
    {
        delete pt;
    }
}

RRT::Point *RRT::Tree::nearest(Geometry::Point2d pt)
{
    bool first = true;
    float bestDistance = 0;
    Point *best = 0;
    BOOST_FOREACH(Point *other, points)
    {
        float d = (other->pos - pt).magsq();
        if (first || d < bestDistance)
        {
            first = false;
            bestDistance = d;
            best = other;
        }
    }
    
    return best;
}

RRT::Point *RRT::Tree::move(Point *start, Geometry::Point2d dest)
{
    Geometry::Point2d delta = dest - start->pos;
    float d = delta.mag();
    
    Geometry::Point2d pos;
    if (d < step)
    {
        pos = dest;
    } else {
        pos = start->pos + delta / d * step;
    }
    
    // Check curvature
    if (start->parent)
    {
        Geometry::Point2d va = (pos - start->pos).norm();
        Geometry::Point2d vb = (start->parent->pos - start->pos).norm();
        float dot = va.dot(vb);
        if (dot >= -1 && dot <= 1)
        {
            // Rounding error for small or zero moves can produce an unusable dot product, so ignore those cases.
            
            float angle = fabs(acos(dot) * 180.0 / M_PI);
            if (angle < 120)
            {
                // Corner too sharp
                return 0;
            }
        }
    }
    
    // Check for obstacles
    if (_obstacles.hit(Geometry::Segment(pos, start->pos)))
    {
        return 0;
    }
    
    return new Point(pos, start);
}

RRT::Tree::Status RRT::Tree::extend(Geometry::Point2d pt, Point **qnew)
{
    Point *near = nearest(pt);
    if (!near)
    {
        // Should never happen - there must be at least one point in the tree
        return Trapped;
    }
    
    Point *newPoint = move(near, pt);
    if (qnew)
    {
        *qnew = newPoint;
    }
    
    if (newPoint)
    {
        points.push_back(newPoint);
        near->edges.push_back(newPoint);
        
        if (newPoint->pos == pt)
        {
            return Reached;
        } else {
            return Advanced;
        }
    } else {
        return Trapped;
    }
}

RRT::Tree::Status RRT::Tree::connect(Geometry::Point2d pt)
{
    Status s;
    do
    {
        s = extend(pt);
    } while (s == Advanced);
    
    return s;
}

bool RRT::plan(const ObstacleSet &obstacles, Geometry::Point2d start, Geometry::Point2d goal, int n, Path &path, SystemState *state)
{
    Tree t0(obstacles, start);
    Tree t1(obstacles, goal);
    
    Tree *ta = &t0;
    Tree *tb = &t1;
    
    for (int i = 0; i < n; ++i)
    {
        Geometry::Point2d r = randomPoint();
        Point *newPoint = 0;
        if (ta->extend(r, &newPoint) != Tree::Trapped)
        {
            if (tb->connect(newPoint->pos) == Tree::Reached)
            {
                findPath(ta, tb, newPoint->pos, path);
                
                state->rrt.clear();
                addEdges(state->rrt, ta->start);
                addEdges(state->rrt, tb->start);
                
                return true;
            }
        }
        
        swap(ta, tb);
    }
    
    return false;
}

void RRT::findPath(Tree *ta, Tree *tb, Geometry::Point2d pt, Path &path)
{
    list<Geometry::Point2d> pa, pb;
    
    find(pa, ta->start, pt);
    find(pb, tb->start, pt);
    pb.pop_front();
    
    path.points.clear();
    
    for (list<Geometry::Point2d>::const_reverse_iterator i = pa.rbegin(); i != pa.rend(); ++i)
    {
        path.points.push_back(*i);
    }

    BOOST_FOREACH(Geometry::Point2d &pt, pb)
    {
        path.points.push_back(pt);
    }
}

void RRT::addEdges(vector<Geometry::Segment> &edges, Point *pt)
{
    BOOST_FOREACH(Point *next, pt->edges)
    {
        edges.push_back(Geometry::Segment(pt->pos, next->pos));
        addEdges(edges, next);
    }
}

bool RRT::find(list<Geometry::Point2d> &path, Point *p, Geometry::Point2d goal)
{
    if (p->pos == goal)
    {
        path.push_back(p->pos);
        return true;
    }
    
    BOOST_FOREACH(Point *next, p->edges)
    {
        if (find(path, next, goal))
        {
            path.push_back(p->pos);
            return true;
        }
    }
    
    return false;
}

float RRT::Path::distance(unsigned int start) const
{
    if (points.empty() || start >= (points.size() - 1))
    {
        return 0;
    }
    
    float distance = 0;
    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        distance += (points[i + 1] - points[i]).mag();
    }
    return distance;
}

bool RRT::Path::hit(const ObstacleSet &obstacles, unsigned int start) const
{
    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        Geometry::Segment seg(points[i], points[i + 1]);
        if (obstacles.hit(seg))
        {
            return true;
        }
    }
    
    return false;
}
