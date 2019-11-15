#pragma once

#include "Planner.hpp"

namespace Planning {

class PathTargetPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    PathTargetPlanner(): counter(0) {}

    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "PathTargetPlanner"; }

    bool shouldReplan(const PlanRequest& request) const override;

    static void createConfiguration(Configuration* cfg);

protected:
    static ConfigDouble* _partialReplanLeadTime;

private:
    std::optional<RJ::Seconds> findInvalidTime(const PlanRequest& request) const;

    int counter;
};

} // namespace Planning
