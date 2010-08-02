#include "Tree.hpp"

#include <iostream>
#include <boost/foreach.hpp>
#include <Utils.hpp>

using namespace Planning;
using namespace std;

//// Point ////
Tree::Point::Point(const Geometry2d::Point& p, Tree::Point* parent) :
	pos(p)
{
	_parent = parent;
	leaf = true;
	
	if (_parent)
	{
		_parent->children.push_back(this);
		_parent->leaf = false;
	}
}

void Tree::Point::addEdges(std::list<Geometry2d::Segment>& edges)
{
	BOOST_FOREACH(Tree::Point* next, children)
	{
		edges.push_back(Geometry2d::Segment(pos, next->pos));
		next->addEdges(edges);
	}
}

//// Tree ////

Tree::Tree()
{
	step = .1;
	_obstacles = 0;
}

Tree::~Tree()
{
	clear();
}

void Tree::clear()
{
	 _obstacles = 0;

    // Delete all points
    BOOST_FOREACH(Point *pt, points)
    {
        delete pt;
    }
    points.clear();
}

void Tree::init(const Geometry2d::Point& start, const ObstacleGroup* obstacles)
{
	clear();
	
	_obstacles = obstacles;
	
	Point* p = new Point(start, 0);
	_obstacles->hit(p->pos, &p->hit);
	points.push_back(p);
}

void Tree::addPath(Planning::Path &path, Point* dest, const bool rev)
{
	list<Point *> points;

	int n = 0;
	while (dest)
	{
		if (rev)
		{
			points.push_back(dest);
		}
		else
		{
			points.push_front(dest);
		}
		dest = dest->parent();
		++n;
	}
	
	path.points.reserve(path.points.size() + n);
	BOOST_FOREACH(Point *pt, points)
	{
		path.points.push_back(pt->pos);
	}
}

Tree::Point* Tree::nearest(Geometry2d::Point pt)
{
	float bestDistance = -1;
    Point *best = 0;
    
    BOOST_FOREACH(Point* other, points)
    {
        float d = (other->pos - pt).magsq();
        if (bestDistance < 0 || d < bestDistance)
        {
            bestDistance = d;
            best = other;
        }
    }

    return best;
}

Tree::Point* Tree::start() const
{
	if (points.empty())
	{
		return 0;
	}
	
	return points.front();
}

Tree::Point* Tree::last() const
{
	if (points.empty())
	{
		return 0;
	}
	
	return points.back();
}

//// Dynamics Tree ////

void DynamicsTree::init(const Geometry2d::Point &pos, const Geometry2d::Point& vel, 
	const ObstacleGroup* obstacles)
{
	Tree::init(pos, obstacles);
	
	//setup initial velocity;
	start()->vel = vel;
	initAngle = 0;
}

Tree::Point* DynamicsTree::extend(Geometry2d::Point pt, Point* base)
{
	//if we don't have a base point, try to find a close point
	//otherwise return
	if (!base)
	{
		base = nearest(pt);
		if (!base)
		{
			return 0;
		}
	}
	
	//time step
	const float deltaT = step;

	//difference between destination and nearest
	const Geometry2d::Point diff = pt - base->pos;
	
	//angle from nearest to destination
	const float angle = Utils::fixAngleDegrees(diff.angle() * RadiansToDegrees - initAngle - 90);
	
	//compute capabilities for our current state
	//angle needs to be in robot space
	//cout << "Getting dynamics info" << endl;
	Dynamics::DynamicsInfo info = dynamics->info(angle, 0); //Segfault here
	//cout << "after Getting dynamics info" << endl;

	const float accel = info.acceleration;
	const float vMax = info.velocity;
	
	Geometry2d::Point deltaV = diff.normalized() * accel * deltaT;
	Geometry2d::Point vFinal = deltaV + base->vel;

	//limit velocity to max in that direction
	if (vFinal.mag() > vMax)
	{
		vFinal = vFinal.normalized() * vMax;
	}
	
	Geometry2d::Point pos = base->pos + vFinal * deltaT;

	// Check for obstacles.

	// moveHit is the set of obstacles that this move touches.
    // If this move touches any obstacles that the starting point didn't already touch,
    // it has entered an obstacle and will be rejected.
    ObstacleSet moveHit;
    if (_obstacles->hit(Geometry2d::Segment(pos, base->pos), &moveHit))
    {
        // We only care if there are any items in moveHit that are not in point->hit, so
        // we don't store the result of set_difference.
        try
        {
            set_difference(moveHit.begin(), moveHit.end(), base->hit.begin(), 
            	base->hit.end(), Utils::ExceptionIterator<ObstaclePtr>());
        } catch (exception e)
        {
            // We hit a new obstacle
            return 0;
        }
    }

    // Allow this point to be added to the tree
	Point* p = new Point(pos, base);
	p->vel = vFinal;
	_obstacles->hit(p->pos, &p->hit);
	points.push_back(p);
	
	return p;
}

//// Fixed Step Tree ////
Tree::Point* FixedStepTree::extend(Geometry2d::Point pt, Tree::Point* base)
{
	//if we don't have a base point, try to find a close point
	//otherwise return
	if (!base)
	{
		base = nearest(pt);
		if (!base)
		{
			return 0;
		}
	}
	
	Geometry2d::Point delta = pt - base->pos;
	float d = delta.mag();
	
	Geometry2d::Point pos;
	if (d < step)
	{
		pos = pt;
	}
	else
	{
		pos = base->pos + delta / d * step;
	}
	
	// Check for obstacles.
	
	// moveHit is the set of obstacles that this move touches.
	// If this move touches any obstacles that the starting point didn't already touch,
	// it has entered an obstacle and will be rejected.
	ObstacleSet moveHit;
	if (_obstacles->hit(Geometry2d::Segment(pos, base->pos), &moveHit))
	{
		// We only care if there are any items in moveHit that are not in point->hit, so
		// we don't store the result of set_difference.
		try
		{
			set_difference(moveHit.begin(), moveHit.end(), base->hit.begin(), 
				base->hit.end(), Utils::ExceptionIterator<ObstaclePtr>());
		} catch (exception e)
		{
			// We hit a new obstacle
			return 0;
		}
	}
	
	// Allow this point to be added to the tree
	Point* p = new Point(pos, base);
	_obstacles->hit(p->pos, &p->hit);
	points.push_back(p);
	
	return p;
}

bool FixedStepTree::connect(Geometry2d::Point pt)
{
	//try to reach the goal pt
	const unsigned int maxAttemps = 50;
	
	Point* from = 0;
	
	for (unsigned int i=0 ; i<maxAttemps ; ++i)
	{
		Point* newPt = extend(pt, from);
		
		//died
		if (!newPt)
		{
			return false;
		}
		
		if (newPt->pos == pt)
		{
			return true;
		}
		
		from = newPt;
	}
	
	return false;
}
