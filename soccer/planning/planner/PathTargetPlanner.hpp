#pragma once

#include "Planner.hpp"
#include <memory>

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner(): counter(0) {}

    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "PathTargetPlanner"; }
    bool shouldReplan(const PlanRequest& request) const override;
    static void createConfiguration(Configuration* cfg);
private:
    static ConfigDouble* _partialReplanLeadTime;
    std::optional<RJ::Seconds> findInvalidTime(const PlanRequest& request) const;

    int counter;
};
} // namespace Planning
