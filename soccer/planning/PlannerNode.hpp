#pragma once

#include <vector>

#include <Context.hpp>
#include "planner/PlanRequest.hpp"
#include "trajectory/Trajectory.hpp"
#include "planner/Planner.hpp"
#include "Node.hpp"

namespace Planning {

class PlannerNode : public Node {
public:
    PlannerNode(Context* context);

    void run() override;

private:
    Context* context_;

    std::vector<std::unique_ptr<Planner>> planners_;

    Trajectory PlanForRobot(Planning::PlanRequest&& request);
};

} // namespace Planning
