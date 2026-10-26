#include "rj_strategy/robot.hpp"

Robot::Robot(int id) { this->id = id; }

bool Robot::equals(Robot other) { return this->id == other.getId(); }

int Robot::getId() { return this->id; }
