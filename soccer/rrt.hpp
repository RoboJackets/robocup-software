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
        Point(Geometry::Point2d pt): pos(pt) {}
        
        Geometry::Point2d pos;
        
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
    void findPath(Tree *ta, Tree *tb, Geometry::Point2d pt, SystemState *state);
    bool plan(const std::vector<Obstacle *> &obstacles, Geometry::Point2d start, Geometry::Point2d goal, int n, SystemState *state);
    void addEdges(std::vector<Geometry::Segment> &edges, Point *pt);
}
