#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include <vector>
#include <rrt/planning/Path.hpp>

namespace Planning {

REGISTER_CONFIGURABLE(PathTargetPlanner);

ConfigDouble* PathTargetPlanner::_partialReplanLeadTime;

void PathTargetPlanner::createConfiguration(Configuration* cfg) {
    _partialReplanLeadTime = new ConfigDouble(
            cfg, "PathTargetPlanner/partialReplanLeadTime", 0.2, "partialReplanLeadTime");
}

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

Trajectory PathTargetPlanner::planWithoutAngles(Planning::PlanRequest &&request) {
    if(!isApplicable(request.motionCommand)) {
        throw std::invalid_argument("Error in PathTargetPlanner: invalid motionCommand; must be a PathTargetCommand.");
    }
    enum ReplanState { Reuse, FullReplan, PartialReplan, CheckBetter };
    ReplanState replanState = Reuse;
    PathTargetCommand& command = std::get<PathTargetCommand>(request.motionCommand);
    RobotInstant startInstant = request.start;
    Trajectory result{{}};

    // Simple case: no path
    const Pose &currentPose = startInstant.pose;
    const Pose &goalPose = command.pathGoal.pose;
    if (currentPose.position() == goalPose.position()) {
        std::vector<RobotInstant> instants;
        instants.push_back(RobotInstant(currentPose, Twist(), RJ::now()));
        result = std::move(Trajectory(std::move(instants)));
        //todo(Ethan) fix this
        result.setDebugText("RRT Basic");
        return std::move(result);
    }

    RJ::Seconds timeIntoTrajectory = RJ::now() - result.begin_time();
    RJ::Seconds invalidTime = 0s;
    if(request.prevTrajectory.empty() || veeredOffPath(request)) {
        replanState = FullReplan;
    } else if (request.prevTrajectory.hit(request.obstacles, timeIntoTrajectory, &invalidTime)) {
        replanState = PartialReplan;
    } else if(goalChanged(request)) {
        invalidTime = request.prevTrajectory.duration();
        replanState = PartialReplan;
    }
    const RJ::Seconds partialReplanTime(*_partialReplanLeadTime);
    if(replanState == PartialReplan) {
        RJ::Seconds timeFromInvalid = invalidTime - timeIntoTrajectory;
        if (timeFromInvalid < partialReplanTime * 2) {
            replanState = FullReplan;
        }
    }

    //assumes the robot is on the path todo(Ethan) ask Kyle about this
//    if (!result.empty()) {
//        auto maybe_start = result.evaluate(RJ::now());
//        if (maybe_start) {
//            startInstant = *maybe_start;
//        }
//    }
    //todo(Ethan) **maybe** delete checkBetter, it's maybe unnecessary
    if(replanState == Reuse) {
        if (counter >= 10) {
            counter = 0;
            if (result.duration() - timeIntoTrajectory > partialReplanTime * 2) {
                replanState = CheckBetter;
            }
        }
        ++counter;//todo(Ethan) this increment should be able to be changed so it's less everywhere
    }

    result = std::move(request.prevTrajectory);
    result.setDebugText("RRT Prev");
    auto stateSpace = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, std::move(request.obstacles));
    if(replanState == CheckBetter || replanState == PartialReplan) {
        Trajectory preTrajectory = result.subTrajectory(0ms, timeIntoTrajectory + partialReplanTime);
        const RobotInstant &middleInstant = preTrajectory.last();
        std::vector<Point> biasWaypoints;
        if(replanState == PartialReplan) {
            biasWaypoints.push_back(startInstant.pose.position() + startInstant.velocity.linear() * 0.2);
            for (auto it = result.iterator(RJ::now(), 100ms); *it != result.last(); ++it) {
                biasWaypoints.push_back((*it).pose.position());
            }
        }
        //todo(Ethan) handle RRT, Bezier, etc. errors generating the path
        std::vector<Point> postPoints = GenerateRRT(middleInstant.pose.position(), goalPose.position(), stateSpace,
                                                    biasWaypoints);
        //todo(ethan) iteratively generate RRTs: the old code would try a bunch of times until it doesn't fail
        if (!postPoints.empty()) {
            RRT::SmoothPath(postPoints, *stateSpace);
            BezierPath postBezier(postPoints, middleInstant.velocity.linear(), command.pathGoal.velocity.linear(),
                                  request.constraints.mot);
            Trajectory postTrajectory = ProfileVelocity(postBezier,
                                                        middleInstant.velocity.linear().mag(),
                                                        command.pathGoal.velocity.linear().mag(),
                                                        request.constraints.mot);
            if(replanState == CheckBetter) {
                RJ::Seconds remaining = result.duration() - (RJ::now() - result.begin_time());
                Trajectory comboPath = Trajectory(std::move(preTrajectory), std::move(postTrajectory));
                if (remaining > comboPath.duration()) {
                    std::cout << "Found A better Path!!!!" << std::endl;
                    return std::move(comboPath);
                } else {
                    replanState = Reuse;
                }
            }
        } else if(replanState == PartialReplan) {
            replanState = FullReplan;
        } else {
            replanState = Reuse;
        }
    }

    if(replanState == FullReplan) {
        std::vector<Point> newPoints = GenerateRRT(currentPose.position(), goalPose.position(), stateSpace);
        if (newPoints.empty()) {
            //            std::cout << "RRT failed (full) " << currentPose.position() << " " << goalPose.position() << std::endl;
            result = Trajectory{{}};
            result.setDebugText("RRT Fail");
            return std::move(result);
        }
        result.setDebugText("RRT Full");
        RRT::SmoothPath(newPoints, *stateSpace);
        BezierPath new_bezier(newPoints, startInstant.velocity.linear(), command.pathGoal.velocity.linear(),
                              request.constraints.mot);
        result = ProfileVelocity(new_bezier,
                startInstant.velocity.linear().mag(),
                command.pathGoal.velocity.linear().mag(),
                request.constraints.mot);
    }
    return std::move(result);
}
Trajectory PathTargetPlanner::plan(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    Trajectory path = planWithoutAngles(std::move(request));
    std::function<double(Point, Point, double)> angleFunction =
            [](Point pos, Point vel_linear, double angle) -> double {
                // Find the nearest angle either matching velocity or at a 180 degree angle.
                //todo(Ethan) fix this, Point heading is radians, `angle` is garbage data
//                double angle_delta = fixAngleRadians(vel_linear.angle() - angle);
//                if (std::abs(angle_delta) < 90) {
//                    return angle + angle_delta;
//                } else {
//                    return fixAngleRadians(angle + angle_delta + M_PI);
//                }
                return vel_linear.angle();
            };
    PlanAngles(path, startInstant, angleFunction, rotationConstraints);
    return std::move(path);
}

bool PathTargetPlanner::goalChanged(const PlanRequest& request) const {
    PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);
    const RobotInstant& last = request.prevTrajectory.last();
    const RobotInstant& goal = command.pathGoal;
    double goalPosDiff = (last.pose.position() - goal.pose.position()).mag();
    double goalVelDiff = (last.velocity.linear() - goal.velocity.linear()).mag();
    return goalPosDiff > Planner::goalPosChangeThreshold()
            || goalVelDiff > Planner::goalVelChangeThreshold();
}

} // namespace Planning
