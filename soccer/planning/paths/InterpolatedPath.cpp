#include "InterpolatedPath.hpp"

#include <rj_robocup_protobuf/LogFrame.pb.h>
#include <utils.h>

#include <stdexcept>

#include "DebugDrawer.hpp"
#include "LogUtils.hpp"
#include "SystemState.hpp"

using namespace std;
using namespace geometry2d;

namespace Planning {

InterpolatedPath::InterpolatedPath(RobotInstant p0) {
    waypoints.emplace_back(p0, RJ::Seconds::zero());
}

InterpolatedPath::InterpolatedPath(RobotInstant p0, RobotInstant p1) {
    waypoints.emplace_back(p0, 0ms);
    waypoints.emplace_back(p1, 0ms);
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
        length +=
            (waypoints[i + 1].pose.position() - waypoints[i].pose.position())
                .mag();
    }
    return length;
}

RobotInstant InterpolatedPath::start() const {
    return RobotInstant(waypoints.front().instant());
}

RobotInstant InterpolatedPath::end() const {
    return RobotInstant(waypoints.back().instant());
}

// Returns the index of the point in this path nearest to pt.
int InterpolatedPath::nearestIndex(Point pt) const {
    if (waypoints.size() == 0) {
        return -1;
    }

    int index = 0;
    float dist = pt.distTo(waypoints[0].pose.position());

    for (unsigned int i = 1; i < waypoints.size(); ++i) {
        float d = pt.distTo(waypoints[i].pose.position());
        if (d < dist) {
            dist = d;
            index = i;
        }
    }

    return index;
}

bool InterpolatedPath::hit(const geometry2d::ShapeSet& obstacles,
                           RJ::Seconds startTimeIntoPath,
                           RJ::Seconds* hitTime) const {
  size_t start = 0;
  for (auto& entry : waypoints) {
    start++;
    if (entry.time > startTimeIntoPath) {
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
      obstacles.hitSet(waypoints[start].pose.position());

  for (size_t i = start; i < waypoints.size() - 1; i++) {
    std::set<std::shared_ptr<Shape>> newHitSet = obstacles.hitSet(Segment(
        waypoints[i].pose.position(), waypoints[i + 1].pose.position()));
    if (!newHitSet.empty()) {
      for (std::shared_ptr<Shape> hit : newHitSet) {
        // If it hits something, check if the hit was in the original
        // hitSet
        if (startHitSet.find(hit) == startHitSet.end()) {
          if (hitTime) {
            *hitTime = waypoints[i].time;
          }
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
        Segment s(waypoints[i].pose.position(),
                  waypoints[i + 1].pose.position());
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
        Segment s(waypoints[i].pose.position(),
                  waypoints[i + 1].pose.position());
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
        Segment s(waypoints[i].pose.position(),
                  waypoints[i + 1].pose.position());

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

void InterpolatedPath::draw(DebugDrawer* const debug_drawer,
                            const QColor& col = Qt::black,
                            const QString& layer = "Motion") const {
    Packet::DebugRobotPath* dbg = debug_drawer->addDebugPath();
    dbg->set_layer(debug_drawer->findDebugLayer(layer));

    if (waypoints.size() <= 1) {
        return;
    }

    for (const Entry& entry : waypoints) {
        Packet::DebugRobotPath::DebugRobotPathPoint* pt = dbg->add_points();
        *pt->mutable_pos() = entry.pose.position();
        *pt->mutable_vel() = entry.vel.linear();
    }
}

std::optional<RobotInstant> InterpolatedPath::eval(RJ::Seconds t) const {
    if (waypoints.size() < 2) {
        return std::nullopt;
    }
    if (t <= waypoints.front().time) {
        return waypoints.front().instant();
    } else if (t > waypoints.back().time) {
        return std::nullopt;
    } else if (t == waypoints.back().time) {
        return waypoints.back().instant();
    }

    // Find the waypoints on either side of the query time such that
    // prev_it->time < t <= next_it->time
    std::vector<Entry>::const_iterator prev_it = waypoints.begin();
    std::vector<Entry>::const_iterator next_it = waypoints.begin();
    while (next_it != waypoints.end()) {
        if (next_it->time >= t) {
            break;
        }

        prev_it = next_it;
        next_it++;
    }

    assert(prev_it != waypoints.end());
    assert(next_it != waypoints.end());

    Entry prev_entry = *prev_it;
    Entry next_entry = *next_it;

    RJ::Seconds dt = next_entry.time - prev_entry.time;
    if (dt == RJ::Seconds(0)) {
        return next_entry.instant();
    }
    RJ::Seconds elapsed = t - prev_entry.time;

    // s in [0, 1] is the interpolation factor.
    double s = elapsed / dt;

    Pose pose_0 = prev_entry.pose;
    Pose pose_1 = next_entry.pose;
    Twist tangent_0 = prev_entry.vel * RJ::numSeconds(dt);
    Twist tangent_1 = next_entry.vel * RJ::numSeconds(dt);

    // Cubic interpolation.
    // We've rescaled the problem to exist in the range [0, 1] instead of
    // [t0, t1] by adjusting the tangent vectors, so now we can interpolate
    // using a Hermite spline. The coefficients for `interpolated_pose` can be
    // found at https://en.wikipedia.org/wiki/Cubic_Hermite_spline. The
    // coefficients for `interpolated_twist` are chosen to be the derivative of
    // `interpolated_pose` with respect to s, and then it is rescaled to match
    // the time derivative
    Pose interpolated_pose =
        Pose(Eigen::Vector3d(pose_0) * (2 * s * s * s - 3 * s * s + 1) +
             Eigen::Vector3d(tangent_0) * (s * s * s - 2 * s * s + s) +
             Eigen::Vector3d(pose_1) * (-2 * s * s * s + 3 * s * s) +
             Eigen::Vector3d(tangent_1) * (s * s * s - s * s));

    Twist interpolated_twist =
        Twist(Eigen::Vector3d(pose_0) * (6 * s * s - 6 * s) +
              Eigen::Vector3d(tangent_0) * (3 * s * s - 4 * s + 1) +
              Eigen::Vector3d(pose_1) * (-6 * s * s + 6 * s) +
              Eigen::Vector3d(tangent_1) * (3 * s * s - 2 * s)) /
        RJ::numSeconds(dt);

    // Create a new MotionInstant
    RobotInstant instant;
    instant.motion.pos = interpolated_pose.position();
    instant.motion.vel = interpolated_twist.linear();
    instant.angle =
        AngleInstant(interpolated_pose.heading(), interpolated_twist.angular());

    return instant;
}

size_t InterpolatedPath::size() const { return waypoints.size(); }

RJ::Seconds InterpolatedPath::getTime(int index) const {
    return waypoints[index].time;
}

RJ::Seconds InterpolatedPath::getDuration() const {
    if (waypoints.size() > 0) {
        return waypoints.back().time - waypoints.front().time;
    } else {
        return RJ::Seconds::zero();
    }
}

unique_ptr<Path> InterpolatedPath::subPath(RJ::Seconds startTime,
                                           RJ::Seconds endTime) const {
    // Check for valid arguments
    if (startTime < RJ::Seconds::zero()) {
        throw invalid_argument("InterpolatedPath::subPath(): startTime(" +
                               to_string(startTime) +
                               ") can't be less than zero");
    }

    if (endTime < RJ::Seconds::zero()) {
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

    if (startTime == RJ::Seconds::zero() && endTime >= getDuration()) {
        return this->clone();
    }

    endTime = std::min(endTime, getDuration());

    auto subpath = make_unique<InterpolatedPath>();

    subpath->addInstant(0s, *eval(startTime));

    // Bound the endTime to a reasonable time.
    endTime = min(endTime, waypoints.back().time);

    // Find the first point in the vector of points which will be included in
    // the subPath. Noninclusive because we always copy eval(startTime)
    auto entry_it = waypoints.begin();
    while (entry_it->time <= startTime) {
        entry_it++;
    }

    // Copy until the time is greater than or equal to endTime. Noninclusive
    // because we always copy eval(endTime)
    while (entry_it != waypoints.end() && entry_it->time < endTime) {
        subpath->waypoints.emplace_back(entry_it->instant(),
                                        entry_it->time - startTime);
        entry_it++;
    }

    subpath->addInstant(endTime - startTime, *eval(endTime));

    return std::move(subpath);
}

unique_ptr<Path> InterpolatedPath::clone() const {
    InterpolatedPath* cp = new InterpolatedPath();
    cp->waypoints = waypoints;
    cp->setStartTime(startTime());
    return std::unique_ptr<Path>(cp);
}

}  // namespace Planning
