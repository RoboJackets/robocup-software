#pragma once

#include <boost/optional.hpp>

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
    
    virtual bool hit(const Geometry2d::Point &pt) const = 0;
    virtual bool hit(const Geometry2d::Segment &seg) const = 0;
};

typedef boost::shared_ptr<Obstacle> ObstaclePtr;

class ObstacleGroup
{
public:
	typedef boost::optional<ObstacleGroup> Optional;

	// STL typedefs
	typedef std::set<ObstaclePtr>::const_iterator const_iterator;
	typedef std::set<ObstaclePtr>::iterator iterator;
	typedef ObstaclePtr value_type;

    ~ObstacleGroup();
    
    const std::set<ObstaclePtr> &obstacles() const
    {
        return _obstacles;
    }
    
    void clear();
    void add(ObstaclePtr obs);
    void add(const ObstacleGroup& group);
    
    // STL Interface
    const_iterator begin() const { return _obstacles.begin(); }
    const_iterator end() const { return _obstacles.end(); }

    iterator begin() { return _obstacles.begin(); }
    iterator end() { return _obstacles.end(); }

    unsigned int size() const
    {
        return _obstacles.size();
    }
    
    bool empty() const
    {
    	return _obstacles.empty();
    }

    template<typename T>
    bool hit(const T &obj, ObstacleGroup &hitSet) const
    {
        for (const_iterator it = begin(); it!=end(); ++it)
        {
            if ((*it)->hit(obj))
            {
				hitSet.add(*it);
            }
        }

        return !hitSet.empty();
    }

    template<typename T>
    bool hit(const T &obj) const
    {
        for (const_iterator it = begin(); it!=end(); ++it)
        {
            if ((*it)->hit(obj))
            {
				return true;
            }
        }

        return false;
    }

protected:
    std::set<ObstaclePtr> _obstacles;
};

class CircleObstacle: public Obstacle
{
public:
    CircleObstacle(Geometry2d::Point center, float radius);
    
    bool hit(const Geometry2d::Point &pt) const;
    bool hit(const Geometry2d::Segment &seg) const;

    Geometry2d::Circle circle;
};

class PolygonObstacle: public Obstacle
{
public:
		PolygonObstacle(){}
		PolygonObstacle(const Geometry2d::Polygon& poly) : polygon(poly) {}

    bool hit(const Geometry2d::Point &pt) const;
    bool hit(const Geometry2d::Segment &seg) const;

    Geometry2d::Polygon polygon;
};
