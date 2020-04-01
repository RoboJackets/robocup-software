#include "RefereeEnums.h"

namespace RefereeModuleEnums {
std::string stringFromStage(Stage s) {
    switch (s) {
        case NORMAL_FIRST_HALF_PRE:
            return "Normal First Half Prep";
        case NORMAL_FIRST_HALF:
            return "Normal First Half";
        case NORMAL_HALF_TIME:
            return "Normal Half Time";
        case NORMAL_SECOND_HALF_PRE:
            return "Normal Second Half Prep";
        case NORMAL_SECOND_HALF:
            return "Normal Second Half";
        case EXTRA_TIME_BREAK:
            return "Extra Time Break";
        case EXTRA_FIRST_HALF_PRE:
            return "Extra First Half Prep";
        case EXTRA_FIRST_HALF:
            return "Extra First Half";
        case EXTRA_HALF_TIME:
            return "Extra Half Time";
        case EXTRA_SECOND_HALF_PRE:
            return "Extra Second Half Prep";
        case EXTRA_SECOND_HALF:
            return "Extra Second Half";
        case PENALTY_SHOOTOUT_BREAK:
            return "Penalty Shootout Break";
        case PENALTY_SHOOTOUT:
            return "Penalty Shootout";
        case POST_GAME:
            return "Post Game";
        default:
            return "";
    }
}

std::string stringFromCommand(Command c) {
    switch (c) {
        case HALT:
            return "Halt";
        case STOP:
            return "Stop";
        case NORMAL_START:
            return "Normal Start";
        case FORCE_START:
            return "Force Start";
        case PREPARE_KICKOFF_YELLOW:
            return "Yellow Kickoff Prep";
        case PREPARE_KICKOFF_BLUE:
            return "Blue Kickoff Prep";
        case PREPARE_PENALTY_YELLOW:
            return "Yellow Penalty Prep";
        case PREPARE_PENALTY_BLUE:
            return "Blue Penalty Prep";
        case DIRECT_FREE_YELLOW:
            return "Direct Yellow Free Kick";
        case DIRECT_FREE_BLUE:
            return "Direct Blue Free Kick";
        case INDIRECT_FREE_YELLOW:
            return "Indirect Yellow Free Kick";
        case INDIRECT_FREE_BLUE:
            return "Indirect Blue Free Kick";
        case TIMEOUT_YELLOW:
            return "Timeout Yellow";
        case TIMEOUT_BLUE:
            return "Timeout Blue";
        case GOAL_YELLOW:
            return "Goal Yellow";
        case GOAL_BLUE:
            return "Goal Blue";
        case BALL_PLACEMENT_YELLOW:
            return "Ball Placement Yellow";
        case BALL_PLACEMENT_BLUE:
            return "Ball Placement Blue";
        default:
            return "";
    }
}
}  // namespace RefereeModuleEnums
