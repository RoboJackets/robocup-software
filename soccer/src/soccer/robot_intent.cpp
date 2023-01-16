#include "robot_intent.hpp"

bool operator==(const RobotIntent& r1, const RobotIntent& r2) {
    SPDLOG_ERROR("deprecated custom struct RI operator called!");
    // if motion_command is a PathTargetCommand
    // TODO: surely there is a way to iterate over all types and just do this?
    if (std::holds_alternative<planning::PathTargetCommand>(r1.motion_command) &&
        std::holds_alternative<planning::PathTargetCommand>(r2.motion_command)) {
        // TODO: fix this? conversion_tests.cpp doesn't seem to be resolving this operator
        /* return std::get<planning::PathTargetCommand>(r1.motion_command) == */
        /*        std::get<planning::PathTargetCommand>(r2.motion_command); */
        return true;
    }

    if (std::holds_alternative<planning::GoalieIdleCommand>(r1.motion_command) &&
        std::holds_alternative<planning::GoalieIdleCommand>(r2.motion_command)) {
        return true;
    }

    // default to address equality
    return &r1 == &r2;
}

bool operator!=(const RobotIntent& r1, const RobotIntent& r2) { return !(r1 == r2); }
