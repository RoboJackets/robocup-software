#include "soccer/src/soccer/robot_intent.hpp"

bool operator==(const RobotIntent& r1, const RobotIntent& r2) {
    // TODO: this is currently ugly, surely there is way to inject the type
    // from the holds_alternative check?

    if (std::holds_alternative<planning::PathTargetCommand>(r1.motion_command) &&
        std::holds_alternative<planning::PathTargetCommand>(r2.motion_command)) {
        planning::PathTargetCommand ptc1 = std::get<planning::PathTargetCommand>(r1.motion_command);
        planning::PathTargetCommand ptc2 = std::get<planning::PathTargetCommand>(r2.motion_command);
        return ptc1 == ptc2;
    }
    if (std::holds_alternative<planning::GoalieIdleCommand>(r1.motion_command) &&
        std::holds_alternative<planning::GoalieIdleCommand>(r2.motion_command)) {
        planning::GoalieIdleCommand gic1 = std::get<planning::GoalieIdleCommand>(r1.motion_command);
        planning::GoalieIdleCommand gic2 = std::get<planning::GoalieIdleCommand>(r2.motion_command);
        return gic1 == gic2;
    }

    // default to address equality
    return &r1 == &r2;
}

bool operator!=(const RobotIntent& r1, const RobotIntent& r2) { return !(r1 == r2); }
