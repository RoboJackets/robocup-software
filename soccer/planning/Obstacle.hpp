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
     * Test whether a given point intersects the obstacle
     */
    virtual bool hit(const Geometry2d::Point &pt) const = 0;
    /**
     * Test whether a given segment intersects the obstacle
     */
    virtual bool hit(const Geometry2d::Segment &seg) const = 0;
};

/**
 * this is a group of obstacles that are all compared to when doing colision detection
 */
class ObstacleGroup
{
public:
	typedef boost::optional<ObstacleGroup> Optional;
    typedef std::shared_ptr<Obstacle> ObstaclePtr;

    ~ObstacleGroup();
    
    /**
     * Returns a std::set of the encapsulated obstacles
     */
    const std::set< ObstaclePtr > &obstacles() const
    {
        return _obstacles;
    }
    
    /**
     * Removes all obstacles
     */
    void clear();

    /**
     * Adds a pointer to an obstacle
     */
    void add(ObstaclePtr obs);

    /**
     * Adds another obstacle group
     */
    void add(const ObstacleGroup& group);
    
    
    unsigned int size() const
    {
        return _obstacles.size();
    }
    
    bool empty() const
    {
    	return _obstacles.empty();
    }

    /**
     * Checks if a given object hits obstacles in the group
     *
     * @param obj The object to collision test
     * @param hitSet A set to add the colliding obstacles to
     * @return A bool telling whether or not there were any collisions
     */
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

    /**
     * Checks if a given obstacle its obstacles in the group
     *
     * @param obj The object to collision test
     * @return A bool telling whether or not there were any collisions
     */
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


    // STL typedefs
    typedef std::set<ObstaclePtr>::const_iterator const_iterator;
    typedef std::set<ObstaclePtr>::iterator iterator;
    typedef ObstaclePtr value_type;
    
    // STL Interface
    const_iterator begin() const { return _obstacles.begin(); }
    const_iterator end() const { return _obstacles.end(); }

    iterator begin() { return _obstacles.begin(); }
    iterator end() { return _obstacles.end(); }


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
