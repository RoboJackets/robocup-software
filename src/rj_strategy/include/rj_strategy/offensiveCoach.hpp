#pragma once
#include <list>

#include "rj_strategy/command.hpp"
#include "rj_strategy/robot.hpp"

namespace commands {
class OffensiveCoach {
public:
    OffensiveCoach();
    void createStrategy(std::list<Robot> robots, bool wePossessBall);

private:
    std::list<Robot> robots;
};
}  // namespace commands