#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include "planning/trajectory/RRTUtil.hpp"

namespace Planning {

Trajectory PathTargetPlanner::checkBetter(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    std::shared_ptr<RoboCupStateSpace> stateSpace = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, std::move(request.static_obstacles));
    const RJ::Seconds timeIntoTrajectory =
            RJ::now() - prevTrajectory.begin_time();
    Trajectory preTrajectory = partialPath(prevTrajectory);
    Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
    PlanAngles(postTrajectory, preTrajectory.last(), angleFunction, request.constraints.rot);
    if (!postTrajectory.empty()) {
        Trajectory comboPath{std::move(preTrajectory),std::move(postTrajectory)};
        if (prevTrajectory.duration() - timeIntoTrajectory > comboPath.duration()) {
            std::cout << "Found A better Path!!!!" << std::endl;
            updatePrevTime(request.shellID);
            return std::move(comboPath);
        }
    }
    return reuse(std::move(request));
}

Trajectory PathTargetPlanner::partialReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    std::vector<Geometry2d::Point> biasWaypoints;
    for (auto it = prevTrajectory.iterator(RJ::now(), 100ms);
         (*it).stamp < prevTrajectory.end_time(); ++it) {
        biasWaypoints.push_back((*it).pose.position());
    }
    Trajectory preTrajectory = partialPath(prevTrajectory);
    Trajectory postTrajectory = RRTTrajectory(preTrajectory.last(), goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles, biasWaypoints);
    if (postTrajectory.empty()) {
        return fullReplan(std::move(request), goalInstant, angleFunction);
    }
    PlanAngles(postTrajectory, preTrajectory.last(), angleFunction, request.constraints.rot);
    Trajectory comboPath = Trajectory(std::move(preTrajectory),
                                      std::move(postTrajectory));
    updatePrevTime(request.shellID);
    return std::move(comboPath);
}

Trajectory PathTargetPlanner::fullReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) {
    Trajectory path = RRTTrajectory(request.start, goalInstant, request.constraints.mot, request.static_obstacles, request.dynamic_obstacles);
    if(path.empty()) {
        return reuse(std::move(request));
    }
    PlanAngles(path, request.start, angleFunction, request.constraints.rot);
    updatePrevTime(request.shellID);
    return std::move(path);
}

}