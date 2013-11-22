#pragma once

#include <boost/optional.hpp>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Circle.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Polygon.hpp>

#include <vector>
#include <set>
#include <memory>

/**
 * Obstacle class handles collision detection
 */
class Obstacle
{
public:
    Obstacle();
    virtual ~Obstacle();
    /**
     * uses to underlying Geom2d class' nearPoint method
     */
    virtual bool hit(const Geometry2d::Point &pt) const = 0;
    /**
     * uses the underlying Geom2D class' nearSegment method
     */
    virtual bool hit(const Geometry2d::Segment &seg) const = 0;
};

typedef std::shared_ptr<Obstacle> ObstaclePtr;
/**
 * this is a group of obstacles that are all compared to when doing colision detection
 */
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
    /**
     * adds a boost shared pointer to an obstacle
     */
    void add(ObstaclePtr obs);
    /**
     * adds another obstacle group
     */
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
/**
 * collision object in the form of a polygon
 */
class PolygonObstacle: public Obstacle
{
public:
		PolygonObstacle(){}
		PolygonObstacle(const Geometry2d::Polygon& poly) : polygon(poly) {}

    bool hit(const Geometry2d::Point &pt) const;
    bool hit(const Geometry2d::Segment &seg) const;

    Geometry2d::Polygon polygon;
};
