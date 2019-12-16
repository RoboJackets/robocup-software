#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include <planning/trajectory/PathSmoothing.hpp>
#include <planning/trajectory/VelocityProfiling.hpp>
#include "PathTargetPlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include <Geometry2d/Pose.hpp>
#include <vector>
#include <rrt/planning/Path.hpp>
#include "Robot.hpp"

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
    const bool veeredOff = veeredOffPath(request);
    Geometry2d::ShapeSet obstacles = std::move(request.obstacles);
    Trajectory prevTrajectory = std::move(request.prevTrajectory);
    PathTargetCommand command = std::get<PathTargetCommand>(std::move(request.motionCommand));
    RobotInstant startInstant = std::move(request.start);
    RobotConstraints constraints = std::move(request.constraints);

    // adjust the requested goal out of an obstacle if necessary
    std::optional<Point> prevGoal;
    if(!prevTrajectory.empty()) {
        prevGoal = prevTrajectory.last().pose.position();
    }
    Point& goalPoint = command.pathGoal.pose.position();
    goalPoint = EscapeObstaclesPathPlanner::findNonBlockedGoal(goalPoint, prevGoal, obstacles);

    // Simple case: no path
    if (startInstant.pose.position() == goalPoint) {
        std::vector<RobotInstant> instants;
        instants.emplace_back(startInstant.pose, Twist(), RJ::now());
        Trajectory result{std::move(instants)};
        result.setDebugText("RRT Basic");
        return std::move(result);
    }


    enum ReplanState { Reuse, FullReplan, PartialReplan, CheckBetter };
    ReplanState replanState = Reuse;
    const RobotInstant& goalInstant = command.pathGoal;
    Trajectory result{{}};

    RJ::Seconds timeIntoTrajectory = RJ::now() - prevTrajectory.begin_time();
    RJ::Seconds invalidTime = 0s;
    if(prevTrajectory.empty() || veeredOff) {
        replanState = FullReplan;
    } else if (prevTrajectory.hit(obstacles, timeIntoTrajectory, &invalidTime)) {
        replanState = PartialReplan;
    } else if(goalChanged(prevTrajectory.last(), goalInstant)) {
        invalidTime = prevTrajectory.duration();
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
        counter++;//todo(Ethan) this increment should be able to be changed so it's less everywhere
    }

    result = std::move(prevTrajectory);
    result.setDebugText("RRT Prev");
    auto stateSpace = std::make_shared<RoboCupStateSpace>(
            Field_Dimensions::Current_Dimensions, std::move(obstacles));
    if(replanState == CheckBetter || replanState == PartialReplan) {
        Trajectory preTrajectory = result.subTrajectory(0ms, timeIntoTrajectory + partialReplanTime);
        RobotInstant middleInstant = preTrajectory.last();
        std::vector<Point> biasWaypoints;
        if(replanState == PartialReplan) {
            biasWaypoints.push_back(startInstant.pose.position() + startInstant.velocity.linear() * 0.2);
            //todo(Ethan) shouldn't this be less than or equal because rn it doesn't add result.end()
            for (auto it = result.iterator(RJ::now(), 100ms); (*it).stamp < result.end_time(); ++it) {
                biasWaypoints.push_back((*it).pose.position());
            }
        }
        //todo(Ethan) handle RRT, Bezier, etc. errors generating the path
        counter++;
        std::vector<Point> postPoints = GenerateRRT(middleInstant.pose.position(), goalInstant.pose.position(), stateSpace, biasWaypoints);
        //todo(ethan) iteratively generate RRTs: the old code would try a bunch of times until it doesn't fail
        if (!postPoints.empty()) {
            RRT::SmoothPath(postPoints, *stateSpace);
            BezierPath postBezier(postPoints, middleInstant.velocity.linear(), goalInstant.velocity.linear(), constraints.mot);
            Trajectory postTrajectory = ProfileVelocity(postBezier,
                                                        middleInstant.velocity.linear().mag(),
                                                        goalInstant.velocity.linear().mag(),
                                                        constraints.mot,
                                                        middleInstant.stamp);
            Trajectory comboPath = Trajectory(std::move(preTrajectory), std::move(postTrajectory));
            if(replanState == CheckBetter) {
                RJ::Seconds remaining = result.duration() - timeIntoTrajectory;
                //todo(Ethan) should be duration of postTrajectory instead of comboPath?
                if (remaining > comboPath.duration()) {
                    std::cout << "Found A better Path!!!!" << std::endl;
                    return std::move(comboPath);
                } else {
                    replanState = Reuse;
                }
            } else {
                return std::move(comboPath);
            }
        } else if(replanState == PartialReplan) {
            replanState = FullReplan;
        } else {
            replanState = Reuse;
        }
    }

    if(replanState == FullReplan) {
        counter++;
        std::vector<Point> newPoints = GenerateRRT(startInstant.pose.position(), goalInstant.pose.position(), stateSpace);
        if (newPoints.empty()) {
            //            std::cout << "RRT failed (full) " << startPose.position() << " " << goalPose.position() << std::endl;
            result = Trajectory{{startInstant}};
            result.setDebugText("RRT Fail");
            return std::move(result);
        }
        RRT::SmoothPath(newPoints, *stateSpace);
        BezierPath new_bezier(newPoints, startInstant.velocity.linear(), goalInstant.velocity.linear(),
                              constraints.mot);
        result = ProfileVelocity(new_bezier,
                                 startInstant.velocity.linear().mag(),
                                 goalInstant.velocity.linear().mag(),
                                 constraints.mot);
        result.setDebugText("RRT Full");
    }
    return std::move(result);
}
Trajectory PathTargetPlanner::plan(PlanRequest&& request) {
    RobotInstant startInstant = request.start;
    RotationConstraints rotationConstraints = request.constraints.rot;
    Trajectory path = planWithoutAngles(std::move(request));
    std::function<double(Point, Point, double)> angleFunction =
            [](Point pos, Point vel_linear, double angle) -> double {
                return vel_linear.angle();
            };
    PlanAngles(path, startInstant, angleFunction, rotationConstraints);
    return std::move(path);
}

bool PathTargetPlanner::goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const {
    double goalPosDiff = (prevGoal.pose.position() - goal.pose.position()).mag();
    double goalVelDiff = (prevGoal.velocity.linear() - goal.velocity.linear()).mag();
    return goalPosDiff > Planner::goalPosChangeThreshold()
            || goalVelDiff > Planner::goalVelChangeThreshold();
}

} // namespace Planning
