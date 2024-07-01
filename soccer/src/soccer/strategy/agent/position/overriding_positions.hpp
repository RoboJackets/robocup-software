#pragma once
#include <map>
#include <string>
#include <vector>

namespace Strategy
{
    /*
        OverridingPositions refers to the positions that can be manually set in the UI.
        Normally, all robots are set to Auto. If you want to add a new position, add a new value
        to this enum before LENGTH, add a string to the overriding_position_labels vector in main_window.hpp,
        and add a case to the check_for_position_override method in RobotFactoryPosition.
    */
    enum OverridingPositions {
        AUTO,
        OFFENSE,
        DEFENSE,
        FREE_KICKER,
        PENALTY_PLAYER,
        PENALTY_NON_KICKER,
        SOLO_OFFENSE,
        SMART_IDLE,
        ZONER,
        IDLE,
        LENGTH, //Do not remove
    };

} // namespace Strategy
