#include "SimplePlanner.hpp"

#include <Sizes.h>

#include <stdio.h>
#include <math.h>
#include <iostream>

using namespace Geometry;
using namespace Packet;
using namespace std;

SimplePlanner::SimplePlanner(Packet::VisionData& vision)
	: PathPlanner(vision)
{
}

PathPlanner::PPOut SimplePlanner::plan(const unsigned int rid, Geometry::Point2d dest)
{
	//clean up vectors
	_paths.clear();
	_orig.clear();
	_dest.clear();
	
	//places to avoid
	//std::vector<Geometry::Circle2d> noZones;
	
	//add predefined no zones
	//noZones = _noZones;
	
	const Point2d pos = _vision.self[rid].pos;
	
	std::vector<Path> possPaths;
	
	//generate nozones for each robot
	for (unsigned int i=0; i<5; ++i)
	{
		VisionData::Robot& opp = _vision.opp[i];
		if (opp.valid)
		{
			_noZones.push_back(Circle2d(opp.pos, ROBOT_RADIUS + .08));
		}
		
		VisionData::Robot& self = _vision.self[i];
		if (self.valid && rid != i) //don't add self as a nozone
		{
			_noZones.push_back(Circle2d(self.pos, ROBOT_RADIUS + .08));
		}
	}
	
	//planned path
	Path path;
	
	//by default the path takes you nowhere
	path.seg1.pt[0] = path.seg1.pt[1] = pos;
	
	bool inZone = false;
	
	//check for avoiding opp half of field
	if (_avoidOpp)
	{
		if (pos.y > (FIELD_LENGTH/2.0f - ROBOT_RADIUS))
		{
			path.seg1.pt[1].y = FIELD_LENGTH/2.0f - ROBOT_RADIUS - .02;
			inZone = true;
		}
		
		if (dest.y > (FIELD_LENGTH/2.0f - ROBOT_RADIUS))
		{
			dest.y = FIELD_LENGTH/2.0f - ROBOT_RADIUS - .02;
		}
	}
	
	for (unsigned int i=0 ; i<_noZones.size() ; ++i)
	{
		//expand each noZone by a robot radius
		//because the robot must not even be in the zone
		_noZones[i].setRadius(_noZones[i].radius() + ROBOT_RADIUS);
		
		//center of the noZone
		Point2d center = Point2d(_noZones[i].center());
		
		const float sourceToCenter = pos.distTo(center);
		const float destToCenter = dest.distTo(center);
		const float radius = _noZones[i].radius(); 
		
		//if we are in a nozone, we have to get out first
		if (sourceToCenter < radius)
		{
			//vector to add = away from center
			//travel enough to get out of zone
			Point2d away = (pos - center).norm(); //move directly out
			away *= radius - sourceToCenter + .02; //extra value to get out of zone
			
			//to pos, add vector to dest
			path.seg1.pt[1] += dest - pos;
			
			//to pos, add vector out
			path.seg1.pt[1] += away;
			
			inZone = true;
		}
	
		//need to move destination point if within a nozone
		if (destToCenter < radius)
		{
			Point2d away = (dest - center).norm();
			away *= radius - destToCenter + .01;
			dest += away;
		}
		
		//generate lines to avoid zone from source and destination point
		genLines(dest, _noZones[i], _dest);
		genLines(pos, _noZones[i], _orig);
	}
	
	//add degenerate
	Path p;
	p.seg1 = Segment(pos, dest);
	possPaths.push_back(p);
	
	if (!inZone)
	{
		//make intersection combinations
		for (unsigned int i=0; i<_orig.size() ; ++i)
		{
			Line2d rLine = _orig[i];
			for (unsigned int j=0; j<_dest.size() ; ++j)
			{
				Line2d dLine = _dest[j];
				
				Point2d iSec;
				
				if (rLine.intersects(dLine, &iSec))
				{
					Segment orig(rLine.pt[0], iSec);
					Segment dest(iSec, dLine.pt[0]);
					
					Path p = { orig, dest };
					possPaths.push_back(p);
				}
			}
		}
		
		//check for noZone intersections
		for (unsigned int i=0; i<possPaths.size() ; ++i)
		{
			Path& p = possPaths[i];
			
			bool add = true;
			for (unsigned int r=0; r<_noZones.size(); ++r)
			{
				if (p.seg1.intersects(_noZones[r]) || p.seg2.intersects(_noZones[r]))
				{
					add = false;
				}
			}
			
			if (add)
			{
				_paths.push_back(p);
			}
		}
		
		path = Path();
		
		if (!_paths.empty())
		{
			path = _paths[0];
			
			for (unsigned int i=1; i<_paths.size() ; ++i)
			{
				if (_paths[i].dist() < path.dist())
				{
					path = _paths[i];
				}
			}
		}
	}
	
	PPOut out;
	out.distance = path.dist();
	//if (rid == 0) printf("%f %f :: %f %f\n", path.seg1.pt[1].x, path.seg1.pt[1].y, path.seg1.pt[0].x, path.seg1.pt[0].y);
	out.direction = (path.seg1.pt[1] - path.seg1.pt[0]).norm();
	
	return out;
}

void SimplePlanner::genLines(const Geometry::Point2d point, 
	const Geometry::Circle2d zone, std::vector<Geometry::Line2d>& vect)
{
	//points in a zone should already be handled
	if (point.distTo(zone.center()) > zone.radius())
	{
		Point2d p1, p2;
		if (zone.tangentPoints(point, &p1, &p2))
		{
			vect.push_back(Line2d(point, p1));
			vect.push_back(Line2d(point, p2));
		}
	}
}
