#pragma once

#include <Eigen/Dense>
#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <time.hpp>

class OurRobot;

/**
 * @brief Takes a trajectory (path) and updates the robot's control packet accordingly.
 */
class TrajectoryExecutor {
public:
    TrajectoryExecutor(OurRobot* robot);

    struct Waypoint {
        Eigen::Vector3d pose, velocity, acceleration;
        RJ::Time last_command_time;
    };

    void stop();

    void run();

    static void createConfiguration(Configuration* cfg);

private:
    OurRobot* robot = nullptr;

    Waypoint last_sent_command;

    static ConfigDouble *max_acceleration, *max_velocity;
};
