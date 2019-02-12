#pragma once

#include <Robot.hpp>
#include <array>

#include <Configuration.hpp>

/**
 * @brief An observation of a robot's position and angle at a certain time
 *
 * @details We get this data out of the packets that come from the visoin sytem.
 * This is fed through RobotFilter, which then sets the position,
 * angle, velocity, and angular velocity of the Robot.
 */
class RobotObservation {
public:
    RobotObservation()
        : pos(), angle(), time(), frameNumber(), valid(false), source() {}

    RobotObservation(Geometry2d::Point pos, float angle, RJ::Time time,
                     int frame, bool valid, int source)
        : pos(pos),
          angle(angle),
          time(time),
          frameNumber(frame),
          valid(valid),
          source(source) {}

    Geometry2d::Point pos;
    float angle;  /// in radians
    RJ::Time time;
    int frameNumber;
    bool valid;
    int source;

    // Compares the times on two observations.  Used for sorting.
    bool operator<(const RobotObservation& other) const {
        return time < other.time;
    }
};

/**
 * adjusts robot vision for accuracy and calculates velocities from positions
 */
class RobotFilter {
public:
    static constexpr int Num_Cameras = 4;

    RobotFilter();

    /// Gives a new observation to the filter
    void update(const std::array<RobotObservation, Num_Cameras>& obs,
                RobotPose* robot, RJ::Time currentTime, u_int32_t frameNumber);

    static void createConfiguration(Configuration* cfg);

private:
    /// Estimate for each camera
    RobotPose _estimates[Num_Cameras];
    RobotPose _currentEstimate;

    static ConfigDouble* _velocity_alpha;
};
