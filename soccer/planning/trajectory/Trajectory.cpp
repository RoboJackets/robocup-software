#include "Trajectory.hpp"
#include <Geometry2d/Shape.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Segment.hpp>
#include <memory>


#include <stdexcept>
#include "Utils.hpp"

namespace Planning {

    using Geometry2d::Pose;
    using Geometry2d::Twist;
    using Geometry2d::Shape;
    using Geometry2d::Segment;

    Trajectory &Trajectory::operator=(Trajectory &&other) {
        //move data into *this
        instants_ = std::move(other.instants_);
        _debugText = std::move(other._debugText);
        angle_override = std::move(other.angle_override);
        //clear data in other
        other.instants_ = {};
        other._debugText = "MOVED FROM";
        other.angle_override = std::nullopt;
        return *this;
    }

    Trajectory &Trajectory::operator=(const Trajectory &other) {
        instants_ = other.instants_;
        _debugText = other._debugText;
        angle_override = other.angle_override;
        return *this;
    }

    Trajectory::Trajectory(const Trajectory &a, const Trajectory &b):
    Trajectory(Trajectory{a}, Trajectory{b}) {}

    Trajectory::Trajectory(Trajectory &&a, Trajectory &&b) {
        instants_ = std::move(a.instants_);
        if(!b.empty()) b.instants_.pop_front();
        instants_.splice(instants_.end(), std::move(b.instants_));
    }

//    void Trajectory::InsertInstant(RobotInstant instant) {
//        instants_.insert(std::upper_bound(
//                instants_.begin(),
//                instants_.end(),
//                instant,
//                [](RobotInstant a, RobotInstant b) {
//                    return a.stamp < b.stamp;
//                }), instant);
//    }

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
        double multiplier = final_duration / duration();

        for (RobotInstant &instant : instants_) {
            instant.velocity /= multiplier;
            instant.stamp = fixed_point +
                            RJ::Seconds(instant.stamp - fixed_point) *
                            multiplier;
        }
    }

    std::optional<RobotInstant>
    Trajectory::evaluate(RJ::Seconds seconds) const {
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

    RobotInstant Trajectory::interpolatedInstant(const RobotInstant& prev_entry, const RobotInstant& next_entry, RJ::Time time) {
        RJ::Seconds dt = next_entry.stamp - prev_entry.stamp;
        if (dt == RJ::Seconds(0)) {
            return next_entry;
        }
        RJ::Seconds elapsed = time - prev_entry.stamp;

        // s in [0, 1] is the interpolation factor.
        double s = elapsed / dt;
        if(std::abs(s) < 1e-6) {
            s = 0;
        }
        if(std::abs(s - 1) < 1e-6) {
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
    //todo(Ethan) make this faster
    bool Trajectory::hit(const Geometry2d::ShapeSet &obstacles,
                         RJ::Seconds startTimeIntoPath,
                         RJ::Seconds *hitTime) const {
        if(empty() || startTimeIntoPath > duration()) {
            return false;
        }
        TrajectoryIterator it = iterator(begin_time() + startTimeIntoPath, (duration() - startTimeIntoPath) * 0.01);

        RobotInstant currentInstant = *it;
        // This code disregards obstacles which the robot starts in. This allows the
        // robot to move out a obstacle if it is already in one.
        std::set<std::shared_ptr<Shape>> startHitSet = obstacles.hitSet(
                currentInstant.pose.position());
        while (it.hasNext()) {
            ++it;
            RobotInstant nextInstant = *it;
            std::set<std::shared_ptr<Shape>> newHitSet = obstacles.hitSet(
                    Segment(currentInstant.pose.position(), nextInstant.pose.position()));
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
        }
        return false;
    }

    bool Trajectory::intersects(const std::vector<DynamicObstacle> &obstacles,
                                RJ::Time startTime,
                                Geometry2d::Point *hitLocation,
                                RJ::Seconds *hitTime) const {
        if(empty()) {
            return false;
        }
        constexpr RJ::Seconds deltaT = 0.05s;
        for (const DynamicObstacle &obs : obstacles) {
            if (obs.path->empty()) {
                Geometry2d::ShapeSet set;
                set.add(obs.circle);
                if (hit(set, startTime - begin_time(), hitTime)) {
                    return true;
                }
            } else {
                for (auto obsIt = obs.path->iterator(startTime, deltaT),
                        thisIt = iterator(startTime, deltaT);
                     obsIt.hasValue() && thisIt.hasValue();
                     ++obsIt, ++thisIt) {
                    Geometry2d::Point obsPoint = (*obsIt).pose.position();
                    double hitRadius = obs.circle->radius() + Robot_Radius;
                    RobotInstant thisInstant = *thisIt;
                    if(thisInstant.pose.position().distTo(obsPoint) < hitRadius) {
                        if(hitTime) {
                            *hitTime = RJ::Seconds(thisInstant.stamp - begin_time());
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
    //todo(Ethan) make this function more efficient, we shouldn't have to copy
    //this will require being careful about how the subtrajectories are used though.
    //and the combo trajectory constructors would need to be changed to l-value ref
    Trajectory Trajectory::subTrajectory(RJ::Seconds startTime,
                                         RJ::Seconds endTime) const {
        // Check for valid arguments
        if (startTime < RJ::Seconds::zero()) {
            throw std::invalid_argument(
                    "InterpolatedPath::subPath(): startTime(" +
                    to_string(startTime) +
                    ") can't be less than zero");
        }
        if (startTime > endTime) {
            throw std::invalid_argument(
                    "InterpolatedPath::subPath(): startTime(" +
                    to_string(startTime) +
                    ") can't be after endTime(" + to_string(endTime) + ")");
        }
        if (startTime >= duration()) {
            debugThrow(std::invalid_argument(
                    "InterpolatedPath::subPath(): startTime(" +
                    to_string(startTime) +
                    ") can't be greater than the duration(" +
                    to_string(duration()) +
                    ") of the path"));
            return Trajectory({});
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
        instants.push_back(interpolatedInstant(instantBeforeStart, instantAfterStart, absolute_start_time));

        // Copy until the time is greater than or equal to endTime. Noninclusive
        // because we always copy eval(endTime)
        RJ::Time absolute_end_time = begin_time() + endTime;
        RobotInstant instantBeforeEnd = *instants_it;
        while ((*instants_it).stamp < absolute_end_time) {
            instants.push_back(*instants_it);
            instantBeforeEnd = *instants_it;
            ++instants_it;
        }
        RobotInstant instantAfterEnd = *instants_it;
        instants.push_back(interpolatedInstant(instantBeforeEnd, instantAfterEnd, absolute_end_time));
        return Trajectory{std::move(instants)};
    }

    Trajectory::TrajectoryIterator
    Trajectory::iterator(RJ::Time startTime, RJ::Seconds deltaT) const {
        return TrajectoryIterator(*this, startTime, deltaT);
    }

    void Trajectory::draw(DebugDrawer *drawer,
                          std::optional<Geometry2d::Point> backupTextPos) const {
        if (instants_.size() > 1) {
            Packet::DebugRobotPath *dbgPath = drawer->getLogFrame()->add_debug_robot_paths();
            dbgPath->set_layer(drawer->findDebugLayer("Motion"));
            for (const RobotInstant &instant : instants_) {
                Packet::DebugRobotPath::DebugRobotPathPoint *pt = dbgPath->add_points();
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
                             QColor(100, 100, 255, 100));
        }
    }

} // namespace Planning
