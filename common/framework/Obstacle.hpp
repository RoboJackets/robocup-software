#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Polygon.hpp>

#include <vector>
#include <set>

#include <boost/shared_ptr.hpp>

class Obstacle
{
public:
    Obstacle();
    virtual ~Obstacle();
    
    virtual bool hit(const Geometry2d::Point &pt) = 0;
    virtual bool hit(const Geometry2d::Segment &seg) = 0;
};

typedef boost::shared_ptr<Obstacle> ObstaclePtr;
typedef std::set<ObstaclePtr> ObstacleSet;

class ObstacleGroup
{
public:
    ~ObstacleGroup();
    
    const std::vector<ObstaclePtr> &obstacles() const
    {
        return _obstacles;
    }
    
    void clear();
    void add(ObstaclePtr obs);
    
    unsigned int size() const
    {
        return _obstacles.size();
    }
    
    template<typename T>
    bool hit(const T &obj, ObstacleSet *hitSet = 0) const
    {
        for (unsigned int i = 0; i < _obstacles.size(); ++i)
        {
            if (_obstacles[i]->hit(obj))
            {
                if (hitSet)
                {
                    hitSet->insert(_obstacles[i]);
                } else {
                    return true;
                }
            }
        }
        
        return hitSet && !hitSet->empty();
    }

protected:
    std::vector<ObstaclePtr> _obstacles;
};

class CircleObstacle: public Obstacle
{
public:
    CircleObstacle(Geometry2d::Point center, float radius);
    
    bool hit(const Geometry2d::Point &pt);
    bool hit(const Geometry2d::Segment &seg);
    
    Geometry2d::Circle circle;
};

class PolygonObstacle: public Obstacle
{
public:
    bool hit(const Geometry2d::Point &pt);
    bool hit(const Geometry2d::Segment &seg);
    
    Geometry2d::Polygon polygon;
};
