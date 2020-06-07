#pragma once

#include <constants.h>
#include <geometry2d/pose.h>

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
struct RobotState {
  geometry2d::Pose pose;
  geometry2d::Twist velocity;
  RJ::Time timestamp;
  bool visible = false;
  bool velocity_valid = false;
};

struct BallState {
  geometry2d::Point position;
  geometry2d::Point velocity;
};

struct WorldState {
    WorldState() {
        their_robots.resize(Num_Shells);
        our_robots.resize(Num_Shells);
    }

    RobotState& get_robot(bool ours, int shell) {
        if (ours) {
            return our_robots.at(shell);
        } else {
            return their_robots.at(shell);
        }
    }

    const RobotState& get_robot(bool ours, int shell) const {
        if (ours) {
            return our_robots.at(shell);
        } else {
            return their_robots.at(shell);
        }
    }

    std::vector<RobotState> their_robots;
    std::vector<RobotState> our_robots;
    BallState ball;
};
