#include "robot_intent.hpp"

bool operator==(const RobotIntent& r1, const RobotIntent& r2) {
    // matches if robot_ids match and motion_commands match
    // ignores all other fields! (unimportant)
    return (r1.robot_id == r2.robot_id) && (r1.motion_command == r2.motion_command);
}

bool operator!=(const RobotIntent& r1, const RobotIntent& r2) { return !(r1 == r2); }
