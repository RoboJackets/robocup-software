#pragma once
#include <map>
#include <string>
#include <vector>

namespace Strategy {
/*
    OverridingPositions refers to the positions that can be manually set in the UI.
    Normally, all robots are set to Auto. If you want to add a new position, add a new value
    to this enum, add a string to the overriding_position_labels vector in
   main_window.hpp, and add a case to the set_position_override_if_requested method in
   RobotFactoryPosition.
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
};

}  // namespace Strategy
