#include "Planner.hpp"
#include "planning/trajectory/Trajectory.hpp"
namespace Planning {
Trajectory Planner::reuse(PlanRequest&& request) {
    Trajectory& prevTrajectory = request.prevTrajectory;
    if(prevTrajectory.empty()) {
        Trajectory out{{request.start}};
        out.setDebugText("Empty");
        return std::move(out);
    }
    RJ::Seconds timeElapsed = RJ::now() - prevTrajectory.begin_time();
    if(timeElapsed < prevTrajectory.duration()) {
        prevTrajectory.trimFront(timeElapsed);
        Trajectory out = std::move(prevTrajectory);
        out.setDebugText("Reuse");
        return std::move(out);
    }
    Trajectory out{{prevTrajectory.last()}};
    out.setDebugText("Reusing Past End");
    return std::move(out);
}
}