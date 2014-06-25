#pragma once

#include <boost/optional.hpp>

#include <Geometry2d/CompositeShape.hpp>
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
class Obstacle : public Geometry2d::CompositeShape
{
public:
    Obstacle();
    virtual ~Obstacle();
    /**
     * Test whether a given point intersects the obstacle
     */
    virtual bool hit(const Geometry2d::Point &pt) const {
        return containsPoint(pt);
    }
    /**
     * Test whether a given segment intersects the obstacle
     */
    virtual bool hit(const Geometry2d::Segment &seg) const = 0;
};

//  TODO: move this functionality into CompositeShape, then delete the class
/**
 * this is a group of obstacles that are all compared to when doing colision detection
 */
class ObstacleGroup
{
public:
	typedef boost::optional<ObstacleGroup> Optional;

    ~ObstacleGroup();
    
    /**
     * Returns a std::set of the encapsulated obstacles
     */
    const std::set< ObstaclePtr > &obstacles() const
    {
        return _obstacles;
    }
    
    
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
