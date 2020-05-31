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

/**
 * \brief Node that is responsible for running all the planners.
 *
 * Input: Global obstacles
 * Output: context_->trajectories
 */
class PlannerNode : public Node {
public:
    PlannerNode(Context* context);

    /**
     * \brief Collects global obstacles, builds plan request per robot, runs
     * the planner, builds a Trajectory and sets context_->trajectories
     */
    void run() override;

private:
    Context* context_;
    std::unique_ptr<Planning::MultiRobotPathPlanner> path_planner_;

    GlobalObstacles getGlobalObstacles();
    std::map<int, Planning::PlanRequest> buildPlanRequests(
        const GlobalObstacles& global_obstacles);
};
}  // namespace Planning
