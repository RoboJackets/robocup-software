#pragma once

#include <list>
#include <vector>
#include <Geometry/Point2d.hpp>
#include <framework/SystemState.hpp>

namespace RRT
{
    Geometry::Point2d randomPoint();
    
    class Point
    {
    public:
        Point(Geometry::Point2d pt, Point *parent = 0): pos(pt), parent(parent) {}
        
        Geometry::Point2d pos;
        
        // The point with an edge leading to this one
        Point *parent;
        
        std::list<Point *> edges;
    };
    
    class Obstacle
    {
    public:
        virtual ~Obstacle();
        
        virtual bool hit(const Geometry::Segment &seg);
    };
    
    class CircleObstacle: public Obstacle
    {
    public:
        CircleObstacle(Geometry::Point2d center, float radius);
    
        virtual bool hit(const Geometry::Segment &seg);
        
        const Geometry::Point2d &center() const
        {
            return _center;
        }
        
        float radius() const
        {
            return _radius;
        }
        
    protected:
        Geometry::Point2d _center;
        float _radius;
    };
    
    class Path
    {
    public:
        // Returns the length of the path starting at point (start).
        float distance(unsigned int start = 0) const;
        
        bool hit(const std::vector<Obstacle *> &obstacles, unsigned int start = 0) const;
        
        std::vector<Geometry::Point2d> points;
    };
    
    class Tree
    {
    public:
        Tree(const std::vector<Obstacle *> &obstacles, Geometry::Point2d pt);
        ~Tree();
        
        Point *nearest(Geometry::Point2d pt);
        Point *move(Point *start, Geometry::Point2d dest);
        
        enum Status
        {
            Trapped,
            Reached,
            Advanced
        };
        
        // qnew optionally points to a (Point *) to receive a pointer to the new point.
        Status extend(Geometry::Point2d pt, Point **qnew = 0);
        
        Status connect(Geometry::Point2d pt);
        
        Point *start;
        std::list<Point *> points;
        float step;
    
    protected:
        const std::vector<Obstacle *> &_obstacles;
    };
    
    bool find(std::list<Geometry::Point2d> &path, Point *p, Geometry::Point2d goal);
    void findPath(Tree *ta, Tree *tb, Geometry::Point2d pt, Path &path);
    bool plan(const std::vector<Obstacle *> &obstacles, Geometry::Point2d start, Geometry::Point2d goal, int n, Path &path, SystemState *state);
    void addEdges(std::vector<Geometry::Segment> &edges, Point *pt);
}
