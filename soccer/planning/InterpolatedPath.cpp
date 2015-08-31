#include "InterpolatedPath.hpp"
#include "LogUtils.hpp"
#include "Utils.hpp"
#include <protobuf/LogFrame.pb.h>

#include <stdexcept>

using namespace std;
using namespace Geometry2d;

namespace Planning {

InterpolatedPath::InterpolatedPath(Point p0) {
    waypoints.emplace_back(MotionInstant(p0, Point()), 0);
}

InterpolatedPath::InterpolatedPath(Point p0, Point p1) {
    waypoints.emplace_back(MotionInstant(p0, Point()), 0);
    waypoints.emplace_back(MotionInstant(p1, Point()), 1);
}

float InterpolatedPath::length(unsigned int start) const {
    return length(start, waypoints.size() - 1);
}

float InterpolatedPath::length(unsigned int start, unsigned int end) const {
    if (waypoints.empty() || start >= (waypoints.size() - 1)) {
        return 0;
    }

    float length = 0;
    for (unsigned int i = start; i < end; ++i) {
        length += (waypoints[i + 1].pos() - waypoints[i].pos()).mag();
    }
    return length;
}

boost::optional<MotionInstant> InterpolatedPath::start() const {
    if (waypoints.empty())
        return boost::none;
    else
        return waypoints.front().instant;
}

boost::optional<MotionInstant> InterpolatedPath::destination() const {
    if (waypoints.empty())
        return boost::none;
    else
        return waypoints.back().instant;
}

// Returns the index of the point in this path nearest to pt.
int InterpolatedPath::nearestIndex(Point pt) const {
    if (waypoints.size() == 0) {
        return -1;
    }

    int index = 0;
    float dist = pt.distTo(waypoints[0].pos());

    for (unsigned int i = 1; i < waypoints.size(); ++i) {
        float d = pt.distTo(waypoints[i].pos());
        if (d < dist) {
            dist = d;
            index = i;
        }
    }

    return index;
}

bool InterpolatedPath::hit(const ShapeSet& obstacles, float& hitTime,
                           float startTime) const {
    size_t start = 0;
    for (auto& entry : waypoints) {
        start++;
        if (entry.time > startTime) {
            start--;
            break;
        }
    }

    if (start >= waypoints.size()) {
        // Empty path or starting beyond end of path
        return false;
    }

    // This code disregards obstacles which the robot starts in. This allows the
    // robot to move out a obstacle if it is already in one.
    std::set<std::shared_ptr<Shape>> startHitSet =
        obstacles.hitSet(waypoints[start].pos());

    for (size_t i = start; i < waypoints.size() - 1; i++) {
        std::set<std::shared_ptr<Shape>> newHitSet = obstacles.hitSet(
            Segment(waypoints[i].pos(), waypoints[i + 1].pos()));
        if (!newHitSet.empty()) {
            for (std::shared_ptr<Shape> hit : newHitSet) {
                // If it hits something, check if the hit was in the original
                // hitSet
                if (startHitSet.find(hit) == startHitSet.end()) {
                    hitTime = waypoints[i].time;
                    return true;
                }
            }
        }
    }
    return false;
}

float InterpolatedPath::distanceTo(Point pt) const {
    int i = nearestIndex(pt);
    if (i < 0) {
        return 0;
    }

    float dist = -1;
    for (unsigned int i = 0; i < (waypoints.size() - 1); ++i) {
        Segment s(waypoints[i].pos(), waypoints[i + 1].pos());
        const float d = s.distTo(pt);

        if (dist < 0 || d < dist) {
            dist = d;
        }
    }

    return dist;
}

Segment InterpolatedPath::nearestSegment(Point pt) const {
    Segment best;
    float dist = -1;
    if (waypoints.empty()) {
        return best;
    }

    for (unsigned int i = 0; i < (waypoints.size() - 1); ++i) {
        Segment s(waypoints[i].pos(), waypoints[i + 1].pos());
        const float d = s.distTo(pt);

        if (dist < 0 || d < dist) {
            best = s;
            dist = d;
        }
    }

    return best;
}

float InterpolatedPath::length(Point pt) const {
    float dist = -1;
    float length = 0;
    if (waypoints.empty()) {
        return 0;
    }

    for (unsigned int i = 0; i < (waypoints.size() - 1); ++i) {
        Segment s(waypoints[i].pos(), waypoints[i + 1].pos());

        // add the segment length
        length += s.length();

        const float d = s.distTo(pt);

        // if point closer to this segment
        if (dist < 0 || d < dist) {
            // closest point on segment
            Point p = s.nearestPoint(pt);

            // new best distance
            dist = d;

            // reset running length count
            length = 0;
            length += p.distTo(s.pt[1]);
        }
    }

    return length;
}

void InterpolatedPath::draw(SystemState* const state,
                            const QColor& col = Qt::black,
                            const QString& layer = "Motion") const {
    Packet::DebugPath* dbg = state->logFrame->add_debug_paths();
    dbg->set_layer(state->findDebugLayer(layer));
    for (const Entry& entry : waypoints) {
        *dbg->add_points() = entry.pos();
    }
    dbg->set_color(color(col));
    return;
}

boost::optional<MotionInstant> InterpolatedPath::evaluate(float t) const {
    if (t < 0) {
        debugThrow(
            invalid_argument("A time less than 0 was entered for time t."));
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
        linearPos,      //  these are set by reference since C++ can't return
    multiple values
        linearSpeed);   //

    Point direction;
    if(!getPoint(linearPos, targetPosOut, direction)) {
        return false;
    }

    targetVelOut = direction * linearSpeed;
    */
    if (waypoints.size() == 0 || waypoints.size() == 1) {
        return boost::none;
    }
    if (t < waypoints[0].time) {
        debugThrow(
            invalid_argument("The start time should not be less than zero"));
    }

    int i = 0;
    while (waypoints[i].time <= t) {
        if (waypoints[i].time == t) {
            return waypoints[i].instant;
        }
        i++;
        if (i == size()) {
            return boost::none;
        }
    }
    float deltaT = (waypoints[i].time - waypoints[i - 1].time);
    if (deltaT == 0) {
        return waypoints[i].instant;
    }
    float constant = (t - waypoints[i - 1].time) / deltaT;

    return MotionInstant(waypoints[i - 1].pos() * (1 - constant) +
                             waypoints[i].pos() * (constant),
                         waypoints[i - 1].vel() * (1 - constant) +
                             waypoints[i].vel() * (constant));
}

size_t InterpolatedPath::size() const { return waypoints.size(); }

bool InterpolatedPath::valid() const { return !waypoints.empty(); }

float InterpolatedPath::getTime(int index) const {
    return waypoints[index].time;
}

float InterpolatedPath::getDuration() const {
    if (waypoints.size() > 0) {
        return waypoints.back().time;
    } else {
        return 0;
    }
}

unique_ptr<Path> InterpolatedPath::subPath(float startTime,
                                           float endTime) const {
    // Check for valid arguments
    if (startTime < 0) {
        throw invalid_argument("InterpolatedPath::subPath(): startTime(" +
                               to_string(startTime) +
                               ") can't be less than zero");
    }

    if (endTime < 0) {
        throw invalid_argument("InterpolatedPath::subPath(): endTime(" +
                               to_string(endTime) +
                               ") can't be less than zero");
    }

    if (startTime > endTime) {
        throw invalid_argument(
            "InterpolatedPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be after endTime(" + to_string(endTime) + ")");
    }

    if (startTime >= getDuration()) {
        debugThrow(invalid_argument(
            "InterpolatedPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be greater than the duration(" + to_string(getDuration()) +
            ") of the path"));
        return unique_ptr<Path>(new InterpolatedPath());
    }

    if (startTime == 0 && endTime >= getDuration()) {
        return this->clone();
    }

    InterpolatedPath* subpath = new InterpolatedPath();

    // Bound the endTime to a reasonable time.
    endTime = min(endTime, getDuration());

    // Find the first point in the vector of points which will be included in
    // the subPath
    size_t start = 0;
    while (waypoints[start].time <= startTime) {
        start++;
    }
    start--;

    // Add the first points to the InterpolatedPath
    if (waypoints[start].time == startTime) {
        subpath->waypoints.emplace_back(waypoints[start].instant, 0);
    } else {
        float deltaT = (waypoints[start + 1].time - waypoints[start].time);
        float constant = (waypoints[start + 1].time - startTime) / deltaT;
        Point startPos = waypoints[start + 1].pos() * (1 - constant) +
                         waypoints[start].pos() * (constant);
        Point vi = waypoints[start + 1].vel() * (1 - constant) +
                   waypoints[start].vel() * (constant);

        subpath->waypoints.emplace_back(MotionInstant(startPos, vi), 0);
    }

    // Find the last point in the InterpolatedPath
    Point vf;
    Point endPos;
    size_t end;
    if (endTime >= getDuration()) {
        end = size() - 1;
        vf = waypoints[end].vel();
        endPos = waypoints[end].pos();
    } else {
        end = start + 1;
        while (waypoints[end].time < endTime) {
            end++;
        }
        float deltaT = (waypoints[end].time - waypoints[end - 1].time);
        float constant = (waypoints[end].time - endTime) / deltaT;
        vf = waypoints[end].vel() * (1 - constant) +
             waypoints[end - 1].vel() * (constant);
        endPos = waypoints[end].pos() * (1 - constant) +
                 waypoints[end - 1].pos() * constant;
    }

    // Add all the points in the middle
    size_t i = start + 1;
    while (i < end) {
        subpath->waypoints.emplace_back(waypoints[i].instant,
                                        waypoints[i].time - startTime);
        i++;
    }

    // Add the last point
    subpath->waypoints.emplace_back(MotionInstant(endPos, vf),
                                    endTime - startTime);

    return unique_ptr<Path>(subpath);
}

unique_ptr<Path> InterpolatedPath::clone() const {
    InterpolatedPath* cp = new InterpolatedPath();
    cp->waypoints = waypoints;
    cp->setStartTime(startTime());
    return std::unique_ptr<Path>(cp);
}

}  // namespace Planning
