
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <algorithm>
#include <Eigen/Dense>
#include "RRTPlanner.hpp"

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
		const MotionConstraints &motionConstraints,
		const Geometry2d::CompositeShape *obstacles,
		Planning::Path &path)
{
	Geometry2d::Point goal = *motionConstraints.targetPos;
	_motionConstraints = motionConstraints;
	vi = vel;

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
	newPath.vi = vi;
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
	//if (_bestPath.points.empty() ||
	//		(hit) ||
	//		//(_bestPath.points.front() != _fixedStepTree0.start()->pos) ||
	//		(_bestPath.points.back() != _fixedStepTree1.start()->pos) ||
	//		(newPath.length() < _bestPath.length()))
	//{
		_bestPath = newPath;
	//	return;
	//}
	/*
	else 
	{
		_bestPath.vi = vi;
		cubicBezier(_bestPath, _obstacles);	
	}*/
}

void RRTPlanner::optimize(Planning::Path &path, const Geometry2d::CompositeShape *obstacles)
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
	//quarticBezier(path, obstacles);
	path.maxSpeed = _motionConstraints.maxSpeed;
	path.endSpeed = _motionConstraints.endSpeed;
	path.maxAcceleration = _motionConstraints.maxAcceleration;
	cubicBezier(path, obstacles);
}

Geometry2d::Point pow(Geometry2d::Point &p1, float i)
{
	return Geometry2d::Point(pow(p1.x, i), pow(p1.y, i));
}

void RRTPlanner::quarticBezier (Planning::Path &path, const Geometry2d::CompositeShape *obstacles)
{
	vector<Geometry2d::Point> pts;
	//vector<Geometry2d::Point> reversed;
	//reversed = path.points;
	//reverse(reversed.begin(), reversed.end());
	Geometry2d::Point p0, p1, p2, p3, p4;
	p1 = path.points.back();	

	int interpolations = 10;

	for (int i=path.points.size()-1; i>0; i--) // access by reference to avoid copying
    {
    	p4 = path.points[i];
    	p0 = path.points[i-1];
    	p3 = 2.0*p4 - p1;
    	p2 = p3*2 - p4;
    	p1 = .5*p0 + .5*p2;

    	for (int j=0; j<interpolations; j++)
    	{
    		float t = 1.0-((float)j / (float)(interpolations));
    		Geometry2d::Point temp = pow(1.0-t, 4) * p0 + 4.0* pow(1.0-t, 3)*t*p1 + 6*pow(1.0-t, 2)*pow(t, 2)*p2 
    						+ 4*(1.0-t)*pow(t, 3)*p3 + pow(t, 4)*p4;
    		pts.push_back(temp);
    	}
    	
    }
    pts.push_back(path.points.front());
   	reverse(pts.begin(), pts.end());
    path.points = pts;
}

using namespace Eigen;
//typedef Matrix<double, Dynamic, Dynamic>;
void RRTPlanner::cubicBezier (Planning::Path &path, const Geometry2d::CompositeShape *obstacles)
{
	int length = path.size();
	int curvesNum = length-1;
	//double Vo();
	if (curvesNum <= 0) {
		cout<<"what?";
		//TODO 
		return;
	}

	//TODO: Get the actual values
	Geometry2d::Point vf(0,0);
	
	Geometry2d::Point vi = path.vi;

	vector<double> pointsX(length);
	vector<double> pointsY(length);
	vector<double> ks(length-1);
	vector<double> ks2(length-1);
	
	
	
	for (int i=0; i<length; i++) {
		pointsX[i] = path.points[i].x;
		pointsY[i] = path.points[i].y;
	}
	for (int i=0; i<curvesNum; i++) {
		ks[i] = 1.0/(path.getTime(i+1)-path.getTime(i));

		//cout<<ks[i]<< " ";
		//ks[i]=1;
		ks2[i] = ks[i]*ks[i];
	}
	
	VectorXd solutionX = cubicBezierCalc(vi.x, vf.x, pointsX, ks, ks2);
	VectorXd solutionY = cubicBezierCalc(vi.x, vf.y, pointsY, ks, ks2);
	
	Geometry2d::Point p0, p1, p2, p3;
	vector<Geometry2d::Point> pts;
	vector<Geometry2d::Point> vels;
	vector<float> times;
	const int interpolations = 10;
	double time=0;
	//cout<<curvesNum<<endl;
	//cout<<ks[0]<<endl;
	for (int i=0; i<curvesNum; i++) // access by reference to avoid copying
    {
    	p0 = path.points[i];
    	p3 = path.points[i+1];
    	p1 = Geometry2d::Point(solutionX(i*2),solutionY(i*2));
    	p2 = Geometry2d::Point(solutionX(i*2 + 1),solutionY(i*2 + 1));
    	
    	//cout<<p0 << " "<<p1<<" "<<p2<<" "<<p3<<endl;

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
    		//cout<<temp<<endl;	
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
	//cout<<path.times.size();
    //cout<<path.times.size()<<endl;
	//Matrix<double, Dynamic, Dynamic> RowVector2i(10, 10);
	//m(0,0)
	//Eigen::VectorXd v(10);
	//Eigen::MatrixX3d equations(10); 
}

VectorXd RRTPlanner::cubicBezierCalc (double vi, double vf, vector<double> &points, 
									vector<double> &ks, vector<double> &ks2)
{
	int curvesNum = points.size() - 1;


	if (curvesNum == 1) 
	{
		
		VectorXd vector(2);
		vector[0] = vi/(3*ks[0]) + points[0];
		vector[1] = points[curvesNum] - vf/(3*ks[curvesNum-1]);
		return vector;
	}
	else {
		//cout<<curvesNum<<endl;
		
		int matrixSize = curvesNum*2;
		MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
		VectorXd answer(matrixSize);
		equations(0,0) = 1;
		answer(0) = vi/(3*ks[0]) + points[0];
		equations(1,matrixSize-1) = 1;
		answer(1) = points[curvesNum] - vf/(3*ks[curvesNum-1]);
		
		int i = 2;
		for (int n=0; n<curvesNum-1; n++) 
		{
			equations(i, n*2 + 1) = ks[n];
			equations(i, n*2 + 2) = ks[n+1];
			answer(i) = (ks[n] + ks[n+1]) * points[n + 1];
			i++;
			//cout<<ks[n]<<endl;
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
		//cout<< i <<" "<<matrixSize<<endl;
		//cout<<solution;
		return solution;
		//return VectorXd(1);




	}
}