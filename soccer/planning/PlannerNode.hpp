#pragma once

#include <Context.hpp>
#include <Node.hpp>
#include <planning/PlanRequest.hpp>
#include <planning/planners/MultiRobotPathPlanner.hpp>

namespace Planning {

struct GlobalObstacles {
    Geometry2d::ShapeSet global_obstacles;
    Geometry2d::ShapeSet goal_zones;
    Geometry2d::ShapeSet global_obstacles_with_goal_zones;
};

class PlannerNode : public Node {
public:
    PlannerNode(Context* context);
    void run() override;

private:
    Context* context_;
    std::unique_ptr<Planning::MultiRobotPathPlanner> path_planner_;

    GlobalObstacles getGlobalObstacles();
    std::map<int, Planning::PlanRequest> buildPlanRequests(
        const GlobalObstacles& global_obstacles);
};
}  // namespace Planning
