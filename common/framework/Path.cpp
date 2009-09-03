// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "Path.hpp"
#include "Utils.hpp"

#include <stdexcept>

using namespace std;

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

bool Planning::Path::hit(const ObstacleGroup &obstacles, unsigned int start, bool exitObstacles) const
{
    if (start >= points.size())
    {
        // Empty path or starting beyond end of path
        return false;
    }
    
    // The set of obstacles the starting point was inside of
    ObstacleSet hit;
    obstacles.hit(points[start], &hit);
    
    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        ObstacleSet newHit;
        obstacles.hit(Geometry2d::Segment(points[i], points[i + 1]), &newHit);
        try
        {
            set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), Utils::ExceptionIterator<ObstaclePtr>());
        } catch (exception e)
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
