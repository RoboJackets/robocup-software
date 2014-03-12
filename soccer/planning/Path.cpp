
#include "Path.hpp"
#include "Utils.hpp"
#include "motion/TrapezoidalMotion.hpp"

#include <stdexcept>

using namespace std;

Planning::Path::Path(const Geometry2d::Point& p0) {
	points.push_back(p0);
}

Planning::Path::Path(const Geometry2d::Point& p0, const Geometry2d::Point& p1) {
	points.push_back(p0);
	points.push_back(p1);
}

float Planning::Path::length(unsigned int start) const
{
    if (points.empty() || start >= (points.size() - 1))
    {
        return 0;
    }
    
    float length  = 0;
    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        length += (points[i + 1] - points[i]).mag();
    }
    return length;
}

Geometry2d::Point::Optional Planning::Path::start() const
{
		if (points.empty())
			return boost::none;
		else
			return points.front();
}

Geometry2d::Point::Optional Planning::Path::destination() const
{
		if (points.empty())
			return boost::none;
		else
			return points.back();
}

// Returns the index of the point in this path nearest to pt.
int Planning::Path::nearestIndex(const Geometry2d::Point &pt) const
{
	if (points.size() == 0)
	{
		return -1;
	}
	
	int index = 0;
	float dist = pt.distTo(points[0]);
	
	for (unsigned int i=1 ; i<points.size(); ++i)
	{
		float d = pt.distTo(points[i]);
		if (d < dist)
		{
			dist = d;
			index = i;
		}
	}
	
	return index;
}

bool Planning::Path::hit(const ObstacleGroup &obstacles, unsigned int start) const
{
    if (start >= points.size())
    {
        // Empty path or starting beyond end of path
        return false;
    }
    
    // The set of obstacles the starting point was inside of
    ObstacleGroup hit;
    obstacles.hit(points[start], hit);
    
    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        ObstacleGroup newHit;
        obstacles.hit(Geometry2d::Segment(points[i], points[i + 1]), newHit);
        try
        {
            set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), ExceptionIterator<ObstaclePtr>());
        } catch (exception& e)
        {
            // Going into a new obstacle
            return true;
        }
    }
    
    // Didn't hit anything or never left any obstacle
    return obstacles.hit(points.back());
}

float Planning::Path::distanceTo(const Geometry2d::Point &pt) const
{
    int i = nearestIndex(pt);
    if (i < 0)
    {
        return 0;
    }
    
    float dist = -1;
    for (unsigned int i = 0; i < (points.size() - 1); ++i)
	{
		Geometry2d::Segment s(points[i], points[i+1]);
		const float d = s.distTo(pt);
		
		if (dist < 0 || d < dist)
		{
			dist = d;
		}
	}
    
    return dist;
}

Geometry2d::Segment Planning::Path::nearestSegment(const Geometry2d::Point &pt) const
{
	Geometry2d::Segment best;
	float dist = -1;
	if (points.empty())
	{
		return best;
	}
	
	for (unsigned int i = 0; i < (points.size() - 1); ++i)
    {
		Geometry2d::Segment s(points[i], points[i+1]);
		const float d = s.distTo(pt);
		
		if (dist < 0 || d < dist)
		{
			best = s;
			dist = d;
		}
	}
	
	return best;
}

void Planning::Path::startFrom(const Geometry2d::Point& pt, Planning::Path& result) const {

	// path will start at the current robot pose
	result.clear();
	result.points.push_back(pt);

	if (points.empty())
		return;

	// handle simple paths
	if (points.size() == 1) {
		result.points.push_back(points.front());
		return;
	}

	// find where to start the path
	Geometry2d::Segment close_segment;
	float dist = -1;
	unsigned int i = (points.front().nearPoint(pt, 0.02)) ? 1 : 0;
	vector<Geometry2d::Point>::const_iterator path_start = ++points.begin();
	for (; i < (points.size() - 1); ++i)
	{
		Geometry2d::Segment s(points[i], points[i+1]);
		const float d = s.distTo(pt);
		if (dist < 0 || d < dist)
		{
			close_segment = s;
			dist = d;
		}
	}

	// slice path
	// new path will be pt, [closest point on nearest segment], [i+1 to end]
	if (dist > 0.0 && dist < 0.02) {
		Geometry2d::Point intersection_pt = close_segment.nearestPoint(pt);
		result.points.push_back(intersection_pt);
	}

	result.points.insert(result.points.end(), path_start, points.end());

}

float Planning::Path::length(const Geometry2d::Point &pt) const
{
	float dist = -1;
	float length = 0;
	if (points.empty())
	{
		return 0;
	}
	
	for (unsigned int i = 0; i < (points.size() - 1); ++i)
    {
		Geometry2d::Segment s(points[i], points[i+1]);
				
		//add the segment length
		length += s.length();
		
		const float d = s.distTo(pt);
		
		//if point closer to this segment
		if (dist < 0 || d < dist)
		{
			//closest point on segment
			Geometry2d::Point p = s.nearestPoint(pt);
			
			//new best distance
			dist = d;
			
			//reset running length count
			length = 0;
			length += p.distTo(s.pt[1]);
		}
	}
	
	return length;
}

bool Planning::Path::getPoint(float distance ,Geometry2d::Point &position, Geometry2d::Point &direction) const
{
	if (points.empty())
	{
		return false;
	}
	for (unsigned int i = 0; i < (points.size() - 1); ++i)
    {
    	Geometry2d::Point vector(points[i + 1] - points[i]);
		//Geometry2d::Segment s(points[i], points[i+1]);
		
		float vectorLength = vector.mag();
		distance -= vectorLength;
		
		if(distance<=0) 
		{
			distance += vectorLength;
			position = points[i] + (vector * (distance / vectorLength));
			direction = vector.normalized();
			return true;
		}
	}
	return false;

}

void Planning::Path::setStartSpeed(float speed) {
	startSpeed = speed;
}

void Planning::Path::setEndSpeed(float speed) {
	endSpeed = speed;
}

float Planning::Path::getStartSpeed() const {
	return startSpeed;
}


bool Planning::Path::evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const
{	//static const float Max_Linear_Speed = 0.008 * 511;
	//static const float Max_Angular_Speed = 511 * 0.02 * M_PI;
	//static const float Linear_Start_Speed = 0;
	static const float Max_Linear_Speed = 1.5;
	static const float Max_Acceleration = 2;
	float linearPos;
	float linearSpeed;

	bool isDone = TrapezoidalMotion( length(), Max_Linear_Speed, Max_Acceleration, t, startSpeed, 0, linearPos, linearSpeed);

	Geometry2d::Point direction;
	if(!getPoint(linearPos, targetPosOut, direction))
	{
		return false;
	}

	targetVelOut = direction * linearSpeed;
	return true;
}
