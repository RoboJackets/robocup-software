
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <algorithm>

#include "rrt.hpp"

#include <Constants.hpp>
#include <Utils.hpp>

using namespace std;
using namespace Planning;

Geometry2d::Point Planning::randomPoint()
{
	float x = Floor_Width * (drand48() - 0.5f);
	float y = Floor_Length * drand48() - Field_Border;

	return Geometry2d::Point(x, y);
}

RRTPlanner::RRTPlanner()
{
	_maxIterations = 100;
}

void RRTPlanner::run(
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
		const float newDist = newGoal.distTo(goal) + Robot_Radius;
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
	
	path = _bestPath;
}

void RRTPlanner::makePath()
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

	optimize(newPath, _obstacles);

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

void RRTPlanner::optimize(Planning::Path &path, const ObstacleGroup *obstacles)
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
	ObstacleGroup hit;

	again:
	obstacles->hit(path.points[start], hit);
	pts.push_back(path.points[start]);
	// [start, start + 1] is guaranteed not to have a collision because it's already in the path.
	for (unsigned int end = start + 2; end < path.points.size(); ++end)
	{
		ObstacleGroup newHit;
		obstacles->hit(Geometry2d::Segment(path.points[start], path.points[end]), newHit);
		try
		{
			set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), ExceptionIterator<ObstaclePtr>());
		} catch (exception& e)
		{
			start = end - 1;
			goto again;
		}
	}
	// Done with the path
	pts.push_back(path.points.back());

	path.points = pts;
}
