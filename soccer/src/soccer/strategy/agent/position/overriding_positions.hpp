#pragma once
#include <map>
#include <vector>
#include <string>

namespace Strategy
{
    enum OverridingPositions {
        AUTO = 0,
        OFFENSE = 1,
        DEFENSE = 2,
        FREE_KICKER = 3,
        PENALTY_PLAYER = 4,
        PENALTY_NON_KICKER = 5,
        SOLO_OFFENSE = 6,
        SMART_IDLE = 7,
        ZONER = 8,
        IDLE = 9,
    };

} // namespace Strategy
