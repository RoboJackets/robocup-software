
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "RRTPlanner.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include <Constants.hpp>
#include <Utils.hpp>


using namespace std;
using namespace Planning;



Geometry2d::Point Planning::randomPoint()
{
	float x = Field_Dimensions::Current_Dimensions.FloorWidth() * (drand48() - 0.5f);
	float y = Field_Dimensions::Current_Dimensions.FloorLength() * drand48() - Field_Dimensions::Current_Dimensions.Border();

	return Geometry2d::Point(x, y);
}

RRTPlanner::RRTPlanner()
{
	_maxIterations = 100;
}

Planning::InterpolatedPath* RRTPlanner::run(
		const Geometry2d::Point &start,
		const float angle,
		const Geometry2d::Point &vel,
		const MotionConstraints &motionConstraints,
		const Geometry2d::CompositeShape *obstacles)
{
	Planning::InterpolatedPath *path = new Planning::InterpolatedPath();
	Geometry2d::Point goal = *motionConstraints.targetPos;
	_motionConstraints = motionConstraints;
	vi = vel;

	_obstacles = obstacles;

	// Simple case: no path
	if (start == goal)
	{
		path->points.push_back(start);
		return path;
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
	/*
	if (!obstacles->hit(Geometry2d::Segment(start, _bestGoal)))
	{
		path.points.push_back(start);
		path.points.push_back(_bestGoal);
		_bestPath = path;
		return;
	}
	*/
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
	*path = makePath();

	if (path->points.empty())
	{
		// FIXME: without these two lines, an empty path is returned which causes errors down the line.
		path->points.push_back(start);
	}
	return path;
}
Planning::InterpolatedPath update(
					Planning::InterpolatedPath &origionalPath,
					
					const float angle,
					const Geometry2d::Point& vel,
					const MotionConstraints &motionConstraints,
					const Geometry2d::CompositeShape* obstacles) 
{
	return InterpolatedPath();
}
Planning::InterpolatedPath RRTPlanner::makePath()
{
	Planning::InterpolatedPath newPath;
	

	Tree::Point* p0 = _fixedStepTree0.last();
	Tree::Point* p1 = _fixedStepTree1.last();

	//sanity check
	if (!p0 || !p1 || p0->pos != p1->pos)
	{
		return newPath;
	}

	
	//	extract path from RRTs
	_fixedStepTree0.addPath(newPath, p0);//add the start tree first...normal order (aka from root to p0)
	_fixedStepTree1.addPath(newPath, p1, true);//add the goal tree in reverse (aka p1 to root)

	optimize(newPath, _obstacles, _motionConstraints, vi);

	//TODO evaluate the old path based on the closest segment
	//and the distance to the endpoint of that segment
	//Otherwise, a new path will always be shorter than the old given we traveled some

	/// Conditions to use new path
	/// 1. old path is empty
	/// 2. goal changed
	/// 3. start changed -- maybe (Roman)
	/// 3. new path is better
	/// 4. old path not valid (hits obstacles)

	return newPath;
}

void RRTPlanner::optimize(Planning::InterpolatedPath &path, const Geometry2d::CompositeShape *obstacles, const MotionConstraints &motionConstraints, Geometry2d::Point vi)
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
	std::set<shared_ptr<Geometry2d::Shape> > hit;

	again:
	obstacles->hit(path.points[start], hit);
	pts.push_back(path.points[start]);
// [start, start + 1] is guaranteed not to have a collision because it's already in the path.
	for (unsigned int end = start + 2; end < path.points.size(); ++end)
	{
		std::set<shared_ptr<Geometry2d::Shape> > newHit;
		obstacles->hit(Geometry2d::Segment(path.points[start], path.points[end]), newHit);
		try
		{
			set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), ExceptionIterator<std::shared_ptr<Geometry2d::Shape>>());
		} catch (exception& e)
		{
			start = end - 1;
			goto again;
		}
	}
	// Done with the path
	pts.push_back(path.points.back());
	path.points = pts;
	cubicBezier(path, obstacles, motionConstraints, vi);
}

Geometry2d::Point pow(Geometry2d::Point &p1, float i)
{
	return Geometry2d::Point(pow(p1.x, i), pow(p1.y, i));
}

using namespace Eigen;

float getTime(Planning::InterpolatedPath &path, int index, const MotionConstraints &motionConstraints, float startSpeed, float endSpeed) {
	return Trapezoidal::getTime(path.length(0,index), path.length(), motionConstraints.maxSpeed, motionConstraints.maxAcceleration, startSpeed, endSpeed);
}

//TODO: Use targeted end velocity
void RRTPlanner::cubicBezier (Planning::InterpolatedPath &path, const Geometry2d::CompositeShape *obstacles, const MotionConstraints &motionConstraints, Geometry2d::Point vi)
{
	int length = path.size();
	int curvesNum = length-1;
	if (curvesNum <= 0) {
		//TODO
		return;
	}

	//TODO: Get the actual values
	Geometry2d::Point vf(0,0);
	vector<double> pointsX(length);
	vector<double> pointsY(length);
	vector<double> ks(length-1);
	vector<double> ks2(length-1);



	for (int i=0; i<length; i++) {
		pointsX[i] = path.points[i].x;
		pointsY[i] = path.points[i].y;
	}
	float startSpeed = 0;
	float endSpeed = motionConstraints.endSpeed;
	for (int i=0; i<curvesNum; i++) {
		ks[i] = 1.0/(getTime(path, i+1, motionConstraints, startSpeed, endSpeed)-getTime(path, i, motionConstraints, startSpeed, endSpeed));
		ks2[i] = ks[i]*ks[i];
	}

	VectorXd solutionX = cubicBezierCalc(vi.x, vf.x, pointsX, ks, ks2);
	VectorXd solutionY = cubicBezierCalc(vi.y, vf.y, pointsY, ks, ks2);

	Geometry2d::Point p0, p1, p2, p3;
	vector<Geometry2d::Point> pts;
	vector<Geometry2d::Point> vels;
	vector<float> times;
	const int interpolations = 10;
	double time=0;

	for (int i=0; i<curvesNum; i++) // access by reference to avoid copying
    {
    	p0 = path.points[i];
    	p3 = path.points[i+1];
    	p1 = Geometry2d::Point(solutionX(i*2),solutionY(i*2));
    	p2 = Geometry2d::Point(solutionX(i*2 + 1),solutionY(i*2 + 1));

    	for (int j=0; j<interpolations; j++)
    	{
    		double k = ks[i];
    		float t = (((float)j / (float)(interpolations)));
    		Geometry2d::Point temp = pow(1.0-t, 3) * p0 + 3.0* pow(1.0-t, 2)*t*p1 + 3*(1.0-t)*pow(t, 2)*p2
    						+ pow(t, 3)*p3;
    		pts.push_back(temp);
    		t = ((float)j / (float)(interpolations))/k;
    		//3 k (-(A (-1 + k t)^2) + k t (2 C - 3 C k t + D k t) + B (1 - 4 k t + 3 k^2 t^2))
    		temp = 3*k*(-(p0*pow(-1 + k*t ,2)) + k*t*(2*p2 - 3*p2*k*t + p3*k*t) + p1*(1 - 4*k*t + 3*pow(k,2)*pow(t,2)));
    		vels.push_back(temp);
    		times.push_back(time +  t);
    	}
    	time+= 1.0/ks[i];


    }
    pts.push_back(path.points[length-1]);
    vels.push_back(vf);
    times.push_back(time);
    path.points = pts;
    path.vels = vels;
    path.times = times;
}

VectorXd RRTPlanner::cubicBezierCalc (double vi, double vf, vector<double> &points,
									vector<double> &ks, vector<double> &ks2)
{
	int curvesNum = points.size() - 1;


	if (curvesNum == 1)
	{

		VectorXd vector(2);
		vector[0] = vi/(3.0*ks[0]) + points[0];
		vector[1] = points[curvesNum] - vf/(3*ks[curvesNum-1]);
		return vector;
	}
	else {
		int matrixSize = curvesNum*2;
		MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
		VectorXd answer(matrixSize);
		equations(0,0) = 1;
		answer(0) = vi/(3.0*ks[0]) + points[0];
		equations(1,matrixSize-1) = 1;
		answer(1) = points[curvesNum] - vf/(3*ks[curvesNum-1]);

		int i = 2;
		for (int n=0; n<curvesNum-1; n++)
		{
			equations(i, n*2 + 1) = ks[n];
			equations(i, n*2 + 2) = ks[n+1];
			answer(i) = (ks[n] + ks[n+1]) * points[n + 1];
			i++;
		}


		for (int n=0; n<curvesNum-1; n++)
		{

			equations(i, n*2) = ks2[n];
			equations(i, n*2 + 1) = -2*ks2[n];
			equations(i, n*2 + 2) = 2*ks2[n+1];
			equations(i, n*2 + 3) = -ks2[n+1];
			answer(i) = points[n + 1] * (ks2[n+1] - ks2[n]);
			i++;
		}

		ColPivHouseholderQR<MatrixXd> solver(equations);
		VectorXd solution = solver.solve(answer);
		return solution;
	}
}
