// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdlib.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <algorithm>

#include "rrt.hpp"

#include <Constants.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Planning;

Geometry2d::Point RRT::randomPoint()
{
    float x = Constants::Floor::Width * (drand48() - 0.5f);
    float y = Constants::Floor::Length * drand48() - Constants::Field::Border;
    
    return Geometry2d::Point(x, y);
}

RRT::Planner::Planner()
{
    _maxIterations = 100;
}

void RRT::Planner::run(
		const Geometry2d::Point &start,
		const float angle, 
		const Geometry2d::Point &vel,
		const Geometry2d::Point &goal,
		const ObstacleGroup *obstacles,
		Planning::Path &path)
{
	//clear any old path
	path.clear();
	
	_obstacles = obstacles;

	// Simple case: no path
	if (start == goal)
	{
		path.points.push_back(start);
		_bestPath = path;
		return;
	}
	
	/// Locate a non blocked goal point
	Geometry2d::Point newGoal = goal;
	
	if (obstacles && obstacles->hit(goal))
	{
		FixedStepTree goalTree;
		goalTree.init(goal, obstacles);
		goalTree.step = .1f;
		
		// The starting point is in an obstacle
		// extend the tree until we find an unobstructed point
		for (int i= 0 ; i< 100 ; ++i)
		{
			Geometry2d::Point r = randomPoint();
			
			//extend to a random point
			Tree::Point* newPoint = goalTree.extend(r);
			
			//if the new point is not blocked
			//it becomes the new goal
			if (newPoint && newPoint->hit.empty())
			{
				newGoal = newPoint->pos;
				break;
			}
		}
		
		/// see if the new goal is better than old one
		/// must be at least a robot radius better else the move isn't worth it
		const float oldDist = _bestGoal.distTo(goal);
		const float newDist = newGoal.distTo(goal) + Constants::Robot::Radius;
		if (newDist < oldDist || obstacles->hit(_bestGoal))
		{
			_bestGoal = newGoal;
		}
	}
	else
	{
		_bestGoal = goal;
	}
	
	/// simple case of direct shot
	if (!obstacles->hit(Geometry2d::Segment(start, _bestGoal)))
	{
		path.points.push_back(start);
		path.points.push_back(_bestGoal);
		_bestPath = path;
		return;
	}
	
	_fixedStepTree0.init(start, obstacles);
	_fixedStepTree1.init(_bestGoal, obstacles);
	_fixedStepTree0.step = _fixedStepTree1.step = .15f;
	
	/// run global position best path search
	Tree* ta = &_fixedStepTree0;
	Tree* tb = &_fixedStepTree1;
	
	for (unsigned int i=0 ; i<_maxIterations; ++i)
	{
		Geometry2d::Point r = randomPoint();
		
		Tree::Point* newPoint = ta->extend(r);
		
		if (newPoint)
		{
			//try to connect the other tree to this point
			if (tb->connect(newPoint->pos))
			{
				//trees connected
				//done with global path finding
				//the path is from start to goal
				//makePath will handle the rest
				break;
			}
		}
		
		swap(ta, tb);
	}
	
	//see if we found a better global path
	makePath();
	
	if (_bestPath.points.empty())
	{
		// FIXME: without these two lines, an empty path is returned which causes errors down the line.
		path.points.push_back(start);
		_bestPath = path;
		return;
	}
	
	/// we now have a full path to follow
	/// create a dynamics tree for the robot
	
	std::vector<Geometry2d::Segment> edges;
		
	unsigned int n = _bestPath.points.size();
	
	for (unsigned int i = 0; i < (n - 1); ++i)
	{
		Geometry2d::Segment s(_bestPath.points[i], _bestPath.points[i+1]);
		edges.push_back(s);
	}
	
	Geometry2d::Segment best;
	float dist = -1;
	unsigned int index = 0;
	unsigned int i = 0;
	BOOST_FOREACH(Geometry2d::Segment& s, edges)
	{
		const float d = s.distTo(start);
		
		if (dist < 0 || d < dist)
		{
			index = i;
			best = s;
			dist = d;
		}
		
		i++;
	}
	
	/// dynamics based path
	
	//add one to the segment index for the next best point index
	index += 1;
	
	//get the target point
	Geometry2d::Point next0 = _bestPath.points[index];
	Geometry2d::Point next1 = next0;
	if (++index < n)
	{
		next1 = _bestPath.points[index];
	}
	
	Geometry2d::Point next2 = next1;
	if (++index < n)
	{
		next2 = _bestPath.points[index];
	}
	
	Geometry2d::Point target = next1;
	
	_dynamicsTree.init(start, vel, obstacles);
	_dynamicsTree.step = .15;
	_dynamicsTree.initAngle = angle;
	
	// FIXME: Segfault in this loop
	/// find a path to the goal
    for (unsigned int i=0 ; i<_maxIterations ; ++i)
    {
    	Geometry2d::Point r;
    	
    	//random number to decide where new point comes from
		const float random = drand48();
		
		//probability of picking point on old path
		const float n0Prob = .05;
		const float n1Prob = .1 + n0Prob;
		const float n2Prob = .05 + n1Prob;
		
		if (random < n1Prob)
		{
			r = next0;
		}
		else if (random < n1Prob)
		{
			r = next1;
		}
		else if (random < n2Prob)
		{
			r = next2;
		}
		else
		{
			//use a random field point
			r = randomPoint();
		}
		
		//TODO maybe check for point falling within certain range
		//let us exit early
		_dynamicsTree.extend(r); // Segfault is in here
    }
    
    Tree::Point* bestPoint = _dynamicsTree.nearest(target);
    if (bestPoint)
    {
    	_dynamicsTree.addPath(path, bestPoint);
    }
}

void RRT::Planner::makePath()
{
	Tree::Point* p0 = _fixedStepTree0.last();
	Tree::Point* p1 = _fixedStepTree1.last();
	
	//sanity check
	if (!p0 || !p1 || p0->pos != p1->pos)
	{
		return;
	}
	
	Planning::Path newPath;
	
	//add the start tree first...normal order
	//aka from root to p0
	_fixedStepTree0.addPath(newPath, p0);
	
	//add the goal tree in reverse
	//aka p1 to root
	_fixedStepTree1.addPath(newPath, p1, true);
	
	//if no obstacles, path will already be optimal
	if (_obstacles)
	{
		optimize(newPath, _obstacles);
	}
	
	/// check the path against the old one
	bool hit = (_obstacles) ? _bestPath.hit(*_obstacles) : false;
	
	//TODO evaluate the old path based on the closest segment
	//and the distance to the endpoint of that segment
	//Otherwise, a new path will always be shorter than the old given we traveled some
	
	/// Conditions to use new path
	/// 1. old path is empty
	/// 2. goal changed
	/// 3. start changed -- maybe (Roman)
	/// 3. new path is better
	/// 4. old path not valid (hits obstacles)
	if (_bestPath.points.empty() ||
	   (hit) || 
	   //(_bestPath.points.front() != _fixedStepTree0.start()->pos) ||
	   (_bestPath.points.back() != _fixedStepTree1.start()->pos) ||
	   (newPath.length() < _bestPath.length()))
	{
		_bestPath = newPath;
		return;
	}
}

void RRT::Planner::optimize(Planning::Path &path, const ObstacleGroup *obstacles)
{
	unsigned int start = 0;
	
    if (path.empty())
    {
        // Nothing to do
        return;
    }
    
    vector<Geometry2d::Point> pts;
    pts.reserve(path.points.size());
    
    // Copy all points that won't be optimized
    vector<Geometry2d::Point>::const_iterator begin = path.points.begin();
    pts.insert(pts.end(), begin, begin + start);
    
    // The set of obstacles the starting point was inside of
    ObstacleSet hit;
    
again:
    obstacles->hit(path.points[start], &hit);
    pts.push_back(path.points[start]);
    // [start, start + 1] is guaranteed not to have a collision because it's already in the path.
    for (unsigned int end = start + 2; end < path.points.size(); ++end)
    {
        ObstacleSet newHit;
        obstacles->hit(Geometry2d::Segment(path.points[start], path.points[end]), &newHit);
        try
        {
            set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), Utils::ExceptionIterator<ObstaclePtr>());
        } catch (exception e)
        {
            start = end - 1;
            goto again;
        }
    }
    // Done with the path
    pts.push_back(path.points.back());
    
    path.points = pts;
}

void RRT::Planner::draw(QPainter& painter)
{
	painter.setPen(Qt::gray);
	
	int n = (int)_bestPath.points.size()-1;
	for (int i=0 ; i<n ; ++i)
	{
		painter.drawLine(_bestPath.points[i].toQPointF(),
			_bestPath.points[i+1].toQPointF());
	}
	
	//painter.setPen(Qt::gray);
#if 0
	Tree::Point* start = _dynamicsTree.start();
	if (start)
	{
		list<Geometry2d::Segment> edges;
		start->addEdges(edges);
		BOOST_FOREACH(const Geometry2d::Segment &seg, edges)
		{
			painter.drawLine(seg.pt[0].toQPointF(), seg.pt[1].toQPointF());
		}
	}
#endif
}

