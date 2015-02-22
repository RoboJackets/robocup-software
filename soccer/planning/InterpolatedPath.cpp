#include <protobuf/LogFrame.pb.h>
#include "InterpolatedPath.hpp"
#include "Utils.hpp"
#include "motion/TrapezoidalMotion.hpp"
#include "LogUtils.hpp"
#include <stdexcept>

using namespace std;
using namespace Planning;


#pragma mark InterpolatedPath

Planning::InterpolatedPath::InterpolatedPath(const Geometry2d::Point& p0) {
	points.push_back(p0);
}

Planning::InterpolatedPath::InterpolatedPath(const Geometry2d::Point& p0, const Geometry2d::Point& p1) {
	points.push_back(p0);
	points.push_back(p1);
}

float Planning::InterpolatedPath::length(unsigned int start) const
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

float Planning::InterpolatedPath::length(unsigned int start, unsigned int end) const
{
    if (points.empty() || start >= (points.size() - 1))
    {
        return 0;
    }

    float length  = 0;
    for (unsigned int i = start; i < end; ++i)
    {
        length += (points[i + 1] - points[i]).mag();
    }
    return length;
}

boost::optional<Geometry2d::Point> Planning::InterpolatedPath::start() const
{
		if (points.empty())
			return boost::none;
		else
			return points.front();
}

boost::optional<Geometry2d::Point> Planning::InterpolatedPath::destination() const
{
		if (points.empty())
			return boost::none;
		else
			return points.back();
}

// Returns the index of the point in this path nearest to pt.
int Planning::InterpolatedPath::nearestIndex(const Geometry2d::Point &pt) const
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

bool Planning::InterpolatedPath::hit(const Geometry2d::CompositeShape &obstacles, Geometry2d::Point *startPosition) const
{
	int start;
	if(startPosition) {
		start = nearestIndex(*startPosition);
	} else {
		start = 0;
	}

    if (start >= points.size())
    {
        // Empty path or starting beyond end of path
        return false;
    }

    // The set of obstacles the starting point was inside of
    std::set<std::shared_ptr<Geometry2d::Shape> > hit;
    obstacles.hit(points[start], hit);

    for (unsigned int i = start; i < (points.size() - 1); ++i)
    {
        std::set<std::shared_ptr<Geometry2d::Shape> > newHit;
        obstacles.hit(Geometry2d::Segment(points[i], points[i + 1]), newHit);
        try
        {
            set_difference(newHit.begin(), newHit.end(), hit.begin(), hit.end(), ExceptionIterator<std::shared_ptr<Geometry2d::Shape>>());
        } catch (exception& e)
        {
            // Going into a new obstacle
            return true;
        }
    }

    // Didn't hit anything or never left any obstacle
    return obstacles.hit(points.back());
}

float Planning::InterpolatedPath::distanceTo(const Geometry2d::Point &pt) const
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

Geometry2d::Segment Planning::InterpolatedPath::nearestSegment(const Geometry2d::Point &pt) const
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

void Planning::InterpolatedPath::startFrom(const Geometry2d::Point& pt, Planning::InterpolatedPath& result) const {

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

float Planning::InterpolatedPath::length(const Geometry2d::Point &pt) const
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

bool Planning::InterpolatedPath::getPoint(float distance ,Geometry2d::Point &position, Geometry2d::Point &direction) const
{
	if (distance<=0) {
		position = points.front();
		return false;
	}
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
	position = points.back();
	return false;

}
void Planning::InterpolatedPath::draw(SystemState * const state, const QColor &col = Qt::black, const QString &layer = "Motion") const {
	Packet::DebugPath *dbg = state->logFrame->add_debug_paths();
	dbg->set_layer(state->findDebugLayer(layer));
	for (Geometry2d::Point pt : points)
	{
		*dbg->add_points() = pt;
	}
	dbg->set_color(color(col));
	return;
}

bool Planning::InterpolatedPath::evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const
{
    if (maxSpeed == -1 || maxAcceleration == -1) {
        throw std::runtime_error("You must set maxSpeed and maxAcceleration before calling Path.evaluate()");
    }

    /*
	float linearPos;
	float linearSpeed;
	bool pathIsValid = TrapezoidalMotion(
        length(),
        maxSpeed,
        maxAcceleration,
        t,
        startSpeed,
        endSpeed,
        linearPos,      //  these are set by reference since C++ can't return multiple values
        linearSpeed);   //

	Geometry2d::Point direction;
	if(!getPoint(linearPos, targetPosOut, direction)) {
		return false;
	}

	targetVelOut = direction * linearSpeed;
	*/
	if (times.size()==0) {
		targetPosOut = Geometry2d::Point(0,0);
		targetVelOut = Geometry2d::Point(0,0);
		return false;
	}
	if (times.size()==1) {
		targetPosOut = points[0];
		targetVelOut = Geometry2d::Point(0,0);
		return false;
	}
	int i=0;
	if (t<times[0])
	{
		targetPosOut = points[0];
		targetVelOut = vels[0];
		return false;
	}
	while (times[i]<=t)
	{
		if (times[i]==t) {
			targetPosOut = points[i];
			targetVelOut = vels[i];
			return true;
		}
		i++;
		if (i==size())
		{
			targetPosOut = points[i-1];
			targetVelOut = Geometry2d::Point(0,0);
			return false;
		}
	}
	float deltaT = (times[i] - times[i-1]);
	if (deltaT==0)
	{
		targetPosOut = points[i];
		targetVelOut = vels[i];
		return true;
	}
	float constant = (t-times[i-1])/deltaT;

	targetPosOut = points[i-1]*(1-constant) + points[i]*(constant);
	targetVelOut = vels[i-1]*(1-constant) + vels[i]*(constant);

	return true;
}

float Planning::InterpolatedPath::getTime(int index) const
{
	if (maxSpeed == -1 || maxAcceleration == -1) {
        throw std::runtime_error("You must set maxSpeed and maxAcceleration before calling Path.evaluate()");
    }

	return Trapezoidal::getTime(length(0,index), length(), maxSpeed, maxAcceleration, startSpeed, endSpeed);
}
