#include <protobuf/LogFrame.pb.h>
#include "InterpolatedPath.hpp"
#include "Utils.hpp"
#include "LogUtils.hpp"
#include <stdexcept>

#pragma mark InterpolatedPath

using namespace std;
using namespace Geometry2d;


namespace Planning {
	InterpolatedPath::InterpolatedPath(const Geometry2d::Point &p0) {
		points.push_back(p0);
	}

	InterpolatedPath::InterpolatedPath(const Geometry2d::Point &p0, const Geometry2d::Point &p1) {
		points.push_back(p0);
		points.push_back(p1);
	}

	float InterpolatedPath::length(unsigned int start) const {
		if (points.empty() || start >= (points.size() - 1)) {
			return 0;
		}

		float length = 0;
		for (unsigned int i = start; i < (points.size() - 1); ++i) {
			length += (points[i + 1] - points[i]).mag();
		}
		return length;
	}

	float InterpolatedPath::length(unsigned int start, unsigned int end) const {
		if (points.empty() || start >= (points.size() - 1)) {
			return 0;
		}

		float length = 0;
		for (unsigned int i = start; i < end; ++i) {
			length += (points[i + 1] - points[i]).mag();
		}
		return length;
	}

	boost::optional<Geometry2d::Point> InterpolatedPath::start() const {
		if (points.empty())
			return boost::none;
		else
			return points.front();
	}

	boost::optional<MotionInstant> InterpolatedPath::destination() const {
		if (points.empty())
			return boost::none;
		else
			return MotionInstant(points.back(), vels.back());
	}

	// Returns the index of the point in this path nearest to pt.
	int InterpolatedPath::nearestIndex(const Geometry2d::Point &pt) const {
		if (points.size() == 0) {
			return -1;
		}

		int index = 0;
		float dist = pt.distTo(points[0]);

		for (unsigned int i = 1; i < points.size(); ++i) {
			float d = pt.distTo(points[i]);
			if (d < dist) {
				dist = d;
				index = i;
			}
		}

		return index;
	}

	bool InterpolatedPath::hit(const Geometry2d::CompositeShape &obstacles, float startTime) const {
		int start = 0;
		for (float t: times) {
			start++;
			if (t > startTime) {
				start--;
				break;
			}
		}

		if (start >= points.size()) {
			// Empty path or starting beyond end of path
			return false;
		}

		//This code disreguards obstacles if which we start in. This the robot to move out a obstacle if it is already in one.
		// The set of obstacles the starting point was inside of
		std::set<std::shared_ptr<Geometry2d::Shape>> startHitSet;
		obstacles.hit(points[start], startHitSet);

		for (size_t i = 0; i < points.size() - 1; i++) {
			std::set<std::shared_ptr<Geometry2d::Shape>> newHitSet;
			if (obstacles.hit(Geometry2d::Segment(points[i], points[i+1]), newHitSet)) {
				for (std::shared_ptr<Geometry2d::Shape> hit : newHitSet) {
					//If it hits something, check if the hit was in hte origional hitSet
					if (startHitSet.find(hit) == startHitSet.end()) {
						return true;
					}
				}
			}
		}
		return false;
	}

	float InterpolatedPath::distanceTo(const Geometry2d::Point &pt) const {
		int i = nearestIndex(pt);
		if (i < 0) {
			return 0;
		}

		float dist = -1;
		for (unsigned int i = 0; i < (points.size() - 1); ++i) {
			Geometry2d::Segment s(points[i], points[i + 1]);
			const float d = s.distTo(pt);

			if (dist < 0 || d < dist) {
				dist = d;
			}
		}

		return dist;
	}

	Geometry2d::Segment InterpolatedPath::nearestSegment(const Geometry2d::Point &pt) const {
		Geometry2d::Segment best;
		float dist = -1;
		if (points.empty()) {
			return best;
		}

		for (unsigned int i = 0; i < (points.size() - 1); ++i) {
			Geometry2d::Segment s(points[i], points[i + 1]);
			const float d = s.distTo(pt);

			if (dist < 0 || d < dist) {
				best = s;
				dist = d;
			}
		}

		return best;
	}

	void InterpolatedPath::startFrom(const Geometry2d::Point &pt, InterpolatedPath &result) const {

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
		for (; i < (points.size() - 1); ++i) {
			Geometry2d::Segment s(points[i], points[i + 1]);
			const float d = s.distTo(pt);
			if (dist < 0 || d < dist) {
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

	float InterpolatedPath::length(const Geometry2d::Point &pt) const {
		float dist = -1;
		float length = 0;
		if (points.empty()) {
			return 0;
		}

		for (unsigned int i = 0; i < (points.size() - 1); ++i) {
			Geometry2d::Segment s(points[i], points[i + 1]);

			//add the segment length
			length += s.length();

			const float d = s.distTo(pt);

			//if point closer to this segment
			if (dist < 0 || d < dist) {
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

	bool InterpolatedPath::getPoint(float distance, Geometry2d::Point &position,
											  Geometry2d::Point &direction) const {
		if (distance <= 0) {
			position = points.front();
			return false;
		}
		if (points.empty()) {
			return false;
		}
		for (unsigned int i = 0; i < (points.size() - 1); ++i) {
			Geometry2d::Point vector(points[i + 1] - points[i]);

			float vectorLength = vector.mag();
			distance -= vectorLength;

			if (distance <= 0) {
				distance += vectorLength;
				position = points[i] + (vector * (distance / vectorLength));
				direction = vector.normalized();
				return true;
			}
		}
		position = points.back();
		return false;

	}

	void InterpolatedPath::draw(SystemState *const state, const QColor &col = Qt::black,
										  const QString &layer = "Motion") const {
		Packet::DebugPath *dbg = state->logFrame->add_debug_paths();
		dbg->set_layer(state->findDebugLayer(layer));
		for (Geometry2d::Point pt : points) {
			*dbg->add_points() = pt;
		}
		dbg->set_color(color(col));
		return;
	}

	bool InterpolatedPath::evaluate(float t, MotionInstant &targetMotionInstant) const {
		if (t<0) {
			debugThrow(invalid_argument("A time less than 0 was entered for time t."));
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
		if (times.size() == 0) {
			targetMotionInstant = MotionInstant();
			return false;
		} else if (times.size() == 1) {
			targetMotionInstant = MotionInstant(points[0], vels[0]);
			return false;
		}
		if (t < times[0]) {
			debugThrow(invalid_argument("The start time should not be less than zero"));
		}

		int i = 0;
		while (times[i] <= t) {
			if (times[i] == t) {
				targetMotionInstant = MotionInstant(points[i], vels[i]);
				return true;
			}
			i++;
			if (i == size()) {
				targetMotionInstant = MotionInstant(points[i - 1], vels[i - 1]);
				return false;
			}
		}
		float deltaT = (times[i] - times[i - 1]);
		if (deltaT == 0) {
			targetMotionInstant = MotionInstant(points[i], vels[i]);
			return true;
		}
		float constant = (t - times[i - 1]) / deltaT;

		targetMotionInstant = MotionInstant(points[i - 1] * (1 - constant) + points[i] * (constant),
											vels[i - 1] * (1 - constant) + vels[i] * (constant));
		return true;
	}

	size_t InterpolatedPath::size() const {
		return points.size();
	}

	bool InterpolatedPath::valid() const {
		return !points.empty();
	}

	float InterpolatedPath::getTime(int index) const {
		return times[index];
	}


	float InterpolatedPath::getDuration() const {
		if (times.size() > 0) {
			return times.back();
		} else {
			return 0;
		}
	}

	unique_ptr<Path> InterpolatedPath::subPath(float startTime, float endTime) const {
		//Check for valid arguments
		if (startTime < 0) {
			throw invalid_argument("InterpolatedPath::subPath(): startTime(" + to_string(startTime) + ") can't be less than zero");
		}

		if (endTime < 0) {
			throw invalid_argument("InterpolatedPath::subPath(): endTime(" + to_string(endTime) + ") can't be less than zero");
		}

		if (startTime > endTime) {
			throw invalid_argument("InterpolatedPath::subPath(): startTime(" + to_string(startTime) + ") can't be after endTime(" + to_string(endTime) + ")");
		}

		if (startTime >= getDuration()) {
			debugThrow(invalid_argument("InterpolatedPath::subPath(): startTime(" + to_string(startTime) + ") can't be greater than the duration(" + to_string(getDuration()) + ") of the path"));
			return unique_ptr<Path>(new InterpolatedPath());
		}

		if (startTime == 0 && endTime >= getDuration()) {
			return this->clone();
		}


		InterpolatedPath *path = new InterpolatedPath();

		//Bound the endTime to a reasonable time.
		endTime = min(endTime, getDuration());

		//Find the first point in the vector of points which will be included in the subPath
		size_t start = 0;
		while (times[start] <= startTime) {
			start++;
		}
		start--;

		//Add the first points to the InterpolatedPath
		path->times.push_back(0);
		if (times[start] == startTime) {
			path->points.push_back(points[start]);
			path->vels.push_back(vels[start]);
		} else {
			float deltaT = (times[start + 1] - times[start]);
			float constant = (times[start + 1] - startTime) / deltaT;
			Point startPos = points[start + 1] * (1 - constant) + points[start] * (constant);
			Point vi = vels[start + 1] * (1 - constant) + vels[start] * (constant);
			path->points.push_back(startPos);
			path->vels.push_back(vi);
		}

		//Find the last point in the InterpolatedPath
		Point vf;
		Point endPos;
		size_t end;
		if (endTime >= getDuration()) {
			end = size() - 1;
			vf = vels[end];
			endPos = points[end];
		} else {
			end = start + 1;
			while (times[end] < endTime) {
				end++;
			}
			float deltaT = (times[end] - times[end - 1]);
			float constant = (times[end] - endTime) / deltaT;
			//endTime = times[end];
			vf = vels[end] * (1 - constant) + vels[end - 1] * (constant);
			endPos = points[end] * (1 - constant) + points[end - 1] * (constant);
		}

		//Add all the points in the middle
		size_t i = start + 1;
		while (i < end) {
			path->points.push_back(points[i]);
			path->vels.push_back(vels[i]);
			path->times.push_back(times[i] - startTime);
			i++;
		}

		//Add the last point
		path->points.push_back(endPos);
		path->vels.push_back(vf);
		path->times.push_back(endTime - startTime);

		return unique_ptr<Path>(path);

	}

	unique_ptr<Path> InterpolatedPath::clone() const {
		return unique_ptr<Path>(new InterpolatedPath(*this));
	}
}