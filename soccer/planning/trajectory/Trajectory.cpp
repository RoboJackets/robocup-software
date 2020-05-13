#include "Trajectory.hpp"
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Shape.hpp>
#include <memory>

#include <stdexcept>
#include "Utils.hpp"

namespace Planning {

using Geometry2d::Pose;
using Geometry2d::Segment;
using Geometry2d::Shape;
using Geometry2d::Twist;

Trajectory::Trajectory(Trajectory&& other)
    : instants_(std::move(other.instants_)),
      _debugText(std::move(other._debugText)),
      stamp(other.stamp) {
    other._debugText = std::nullopt;
    other.stamp = RJ::Time{0s};
}

Trajectory& Trajectory::operator=(Trajectory&& other) {
    // move data into *this
    instants_ = std::move(other.instants_);
    _debugText = std::move(other._debugText);
    stamp = other.stamp;
    // clear data in other
    other._debugText = std::nullopt;
    other.stamp = RJ::Time{0s};
    return *this;
}

Trajectory::Trajectory(const Trajectory& a, const Trajectory& b)
    : Trajectory(Trajectory{a}, Trajectory{b}) {}

Trajectory::Trajectory(Trajectory&& a, Trajectory&& b) : stamp(b.stamp) {
    instants_ = std::move(a.instants_);
    if (!b.empty()) b.instants_.pop_front();
    instants_.splice(instants_.end(), std::move(b.instants_));
}

void Trajectory::AppendInstant(RobotInstant instant) {
    assert(empty() || instant.stamp > end_time());
    instants_.push_back(instant);
}

bool Trajectory::CheckTime(RJ::Time time) const {
    return time >= begin_time() && time <= end_time();
}

bool Trajectory::CheckSeconds(RJ::Seconds seconds) const {
    return seconds >= 0s && seconds <= duration();
}

void Trajectory::ScaleDuration(RJ::Seconds final_duration) {
    ScaleDuration(final_duration, begin_time());
}

void Trajectory::ScaleDuration(RJ::Seconds final_duration,
                               RJ::Time fixed_point) {
    assert(fixed_point >= begin_time() && fixed_point <= end_time());
    double multiplier = final_duration / duration();

    for (RobotInstant& instant : instants_) {
        instant.velocity /= multiplier;
        instant.stamp =
            fixed_point + RJ::Seconds(instant.stamp - fixed_point) * multiplier;
    }
}

std::optional<RobotInstant> Trajectory::evaluate(RJ::Seconds seconds) const {
    if (instants_.empty()) {
        return std::nullopt;
    }

    return evaluate(begin_time() + seconds);
}

std::optional<RobotInstant> Trajectory::evaluate(RJ::Time time) const {
    if (instants_.empty() || !CheckTime(time)) {
        return std::nullopt;
    }
    return *iterator(time, 10ms);
}

RobotInstant Trajectory::interpolatedInstant(const RobotInstant& prev_entry,
                                             const RobotInstant& next_entry,
                                             RJ::Time time) {
    assert(time >= prev_entry.stamp);
    assert(time <= next_entry.stamp);
    RJ::Seconds dt = next_entry.stamp - prev_entry.stamp;
    if (dt == RJ::Seconds(0)) {
        return next_entry;
    }
    RJ::Seconds elapsed = time - prev_entry.stamp;

    // s in [0, 1] is the interpolation factor.
    double s = elapsed / dt;
    if (std::abs(s) < 1e-6) {
        s = 0;
    }
    if (std::abs(s - 1) < 1e-6) {
        s = 1;
    }
    assert(s >= 0 && s <= 1);

    Pose pose_0 = prev_entry.pose;
    Pose pose_1 = next_entry.pose;
    Twist tangent_0 = prev_entry.velocity * RJ::numSeconds(dt);
    Twist tangent_1 = next_entry.velocity * RJ::numSeconds(dt);

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

    // Create a new RobotInstant with the correct values.
    return RobotInstant{interpolated_pose, interpolated_twist, time};
}
bool Trajectory::hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime) const {
    if (startTimeIntoPath < 0s) {
        debugThrow("Error in Trajectory::hit(). Invalid start time\n");
    }
    if (empty() || startTimeIntoPath > duration()) {
        return false;
    }
    // this fixes the edge case where we increment by a duration of 0s and
    // end up looping forever TODO do this in a more readable way
    constexpr int max_iterations = 100;
    TrajectoryIterator it =
        iterator(begin_time() + startTimeIntoPath,
                 (duration() - startTimeIntoPath) / (double)max_iterations);

    RobotInstant currentInstant = *it;
    // This code disregards obstacles which the robot starts in. This allows the
    // robot to move out a obstacle if it is already in one.
    std::set<std::shared_ptr<Shape>> startHitSet =
        obstacles.hitSet(currentInstant.pose.position());
    int counter = 0;
    while (it.hasNext()) {
        ++it;
        RobotInstant nextInstant = *it;
        std::set<std::shared_ptr<Shape>> newHitSet = obstacles.hitSet(Segment(
            currentInstant.pose.position(), nextInstant.pose.position()));
        if (!newHitSet.empty()) {
            for (const std::shared_ptr<Shape>& hit : newHitSet) {
                // If it hits something, check if the hit was in the original
                // hitSet
                if (startHitSet.find(hit) == startHitSet.end()) {
                    if (hitTime) {
                        *hitTime = currentInstant.stamp - begin_time();
                    }
                    return true;
                }
            }
        }
        currentInstant = nextInstant;
        counter++;
        if (counter > max_iterations) {
            break;
        }
    }
    return false;
}
bool Trajectory::intersects(const std::vector<DynamicObstacle>& obstacles,
                            RJ::Time startTime, Geometry2d::Point* hitLocation,
                            RJ::Seconds* hitTime) const {
    if (empty()) {
        return false;
    }
    constexpr RJ::Seconds deltaT = 0.05s;
    for (const DynamicObstacle& obs : obstacles) {
        if (obs.path->empty()) {
            Geometry2d::ShapeSet set;
            set.add(obs.circle);
            RJ::Seconds staticHit;  // temporary to prevent side effects
            if (hit(set, startTime - begin_time(), &staticHit)) {
                if (hitTime) {
                    *hitTime = staticHit;
                }
                return true;
            }
        } else {
            for (auto obsIt = obs.path->iterator(startTime, deltaT),
                      thisIt = iterator(startTime, deltaT);
                 obsIt.hasValue() && thisIt.hasValue(); ++obsIt, ++thisIt) {
                Geometry2d::Point obsPoint = (*obsIt).pose.position();
                double hitRadius = obs.circle->radius() + Robot_Radius;
                RobotInstant thisInstant = *thisIt;
                if (thisInstant.pose.position().distTo(obsPoint) < hitRadius) {
                    if (hitTime) {
                        *hitTime =
                            RJ::Seconds(thisInstant.stamp - begin_time());
                    }
                    if (hitLocation) {
                        *hitLocation = obsPoint;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}
Trajectory Trajectory::subTrajectory(RJ::Seconds startTime,
                                     RJ::Seconds endTime) const {
    // Check for valid arguments
    if (startTime < RJ::Seconds::zero()) {
        throw std::invalid_argument("InterpolatedPath::subPath(): startTime(" +
                                    to_string(startTime) +
                                    ") can't be less than zero");
    }
    if (startTime > endTime) {
        throw std::invalid_argument(
            "InterpolatedPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be after endTime(" + to_string(endTime) + ")");
    }
    if (startTime >= duration()) {
        debugThrow(std::invalid_argument(
            "InterpolatedPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be greater than the duration(" + to_string(duration()) +
            ") of the path"));
        return Trajectory();
    }
    if (startTime == RJ::Seconds::zero() && endTime >= duration()) {
        return this->clone();
    }
    endTime = std::min(endTime, duration());
    std::list<RobotInstant> instants{};
    // Find the first point in the vector of points which will be included in
    // the subPath. Noninclusive because we always copy eval(startTime)
    auto instants_it = instants_.begin();
    RJ::Time absolute_start_time = begin_time() + startTime;
    RobotInstant instantBeforeStart;
    while ((*instants_it).stamp <= absolute_start_time) {
        instantBeforeStart = *instants_it;
        ++instants_it;
    }
    RobotInstant instantAfterStart = *instants_it;
    instants.push_back(interpolatedInstant(
        instantBeforeStart, instantAfterStart, absolute_start_time));

    // Copy until the time is greater than or equal to endTime. Noninclusive
    // because we always copy eval(endTime)
    RJ::Time absolute_end_time = begin_time() + endTime;
    RobotInstant instantBeforeEnd = *std::prev(instants_it);
    while ((*instants_it).stamp < absolute_end_time) {
        instants.push_back(*instants_it);
        instantBeforeEnd = *instants_it;
        ++instants_it;
    }
    RobotInstant instantAfterEnd = *instants_it;
    instants.push_back(interpolatedInstant(instantBeforeEnd, instantAfterEnd,
                                           absolute_end_time));
    return Trajectory{std::move(instants)};
}

void Trajectory::trimFront(RJ::Seconds startTime) {
    if (startTime == 0s) {
        return;
    }
    std::optional<RobotInstant> startInstant = evaluate(startTime);
    if (!startInstant) {
        if (startTime > 0s) instants_ = {};
        return;
    }
    while (!instants_.empty() &&
           instants_.front().stamp < startInstant->stamp) {
        instants_.pop_front();
    }
    if (startInstant->stamp < instants_.front().stamp) {
        instants_.push_front(*startInstant);
    }
}

TrajectoryIterator Trajectory::iterator(RJ::Time startTime,
                                        RJ::Seconds deltaT) const {
    return TrajectoryIterator(*this, startTime, deltaT);
}

void Trajectory::draw(DebugDrawer* drawer,
                      std::optional<Geometry2d::Point> backupTextPos) const {
    if (instants_.size() > 1) {
        Packet::DebugRobotPath* dbgPath =
            drawer->getLogFrame()->add_debug_robot_paths();
        dbgPath->set_layer(drawer->findDebugLayer("Motion"));
        for (const RobotInstant& instant : instants_) {
            Packet::DebugRobotPath::DebugRobotPathPoint* pt =
                dbgPath->add_points();
            *pt->mutable_pos() = instant.pose.position();
            *pt->mutable_vel() = instant.velocity.linear();
        }
    }
    if (_debugText) {
        Geometry2d::Point textPos;
        if (empty()) {
            if (backupTextPos) {
                textPos = *backupTextPos;
            } else {
                return;
            }
        } else {
            textPos = first().pose.position() + Geometry2d::Point(0.1, 0);
        }
        drawer->drawText(_debugText.value(), textPos,
                         QColor(100, 100, 255, 100), "PlanningDebugText");
    }
}

TrajectoryIterator::TrajectoryIterator(const Trajectory& trajectory,
                                       RJ::Time startTime, RJ::Seconds deltaT)
    : _trajectory(trajectory), _deltaT(deltaT), _time(startTime) {
    assert(!trajectory.empty());
    if (!trajectory.CheckTime(_time)) return;
    if (startTime < trajectory.begin_time() + trajectory.duration() * 0.5) {
        _iterator = trajectory.instants_begin();
        ++_iterator;
        while (_iterator->stamp <= _time) {
            ++_iterator;
        }
        --_iterator;
    } else {
        _iterator = trajectory.instants_end();
        --_iterator;
        while (_iterator->stamp > _time) {
            --_iterator;
        }
    }
}

RobotInstant TrajectoryIterator::operator*() const {
    assert(hasValue());
    if (!hasNext()) {
        return _trajectory.last();
    }
    return Trajectory::interpolatedInstant(*_iterator, *std::next(_iterator),
                                           _time);
}

void TrajectoryIterator::advance(RJ::Time time) {
    _time = time;
    ++_iterator;
    while (_iterator != _trajectory.instants_end() &&
           _iterator->stamp <= _time) {
        ++_iterator;
    }
    --_iterator;
}
}  // namespace Planning
