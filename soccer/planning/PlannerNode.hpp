#pragma once

#include <Context.hpp>
#include <vector>

#include "Node.hpp"
#include "Trajectory.hpp"
#include "planner/PlanRequest.hpp"
#include "planner/Planner.hpp"

namespace Planning {

class PlannerForRobot {
public:
    PlannerForRobot();

    Trajectory PlanForRobot(const Planning::PlanRequest& request);

private:
    std::vector<std::unique_ptr<Planner>> planners_;
};

class PlannerNode : public Node {
public:
    PlannerNode(Context* context);

    void run() override;

private:
    Context* context_;

    std::vector<PlannerForRobot> robots_planners_;
};

}  // namespace Planning
