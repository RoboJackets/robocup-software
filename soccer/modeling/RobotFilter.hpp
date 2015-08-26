#pragma once

#include <Robot.hpp>

/**
 * @brief An observation of a robot's position and angle at a certain time
 *
 * @details We get this data out of the packets that come from the visoin sytem.
 * This is fed through RobotFilter, which then sets the position,
 * angle, velocity, and angular velocity of the Robot.
 */
class RobotObservation {
public:
    RobotObservation(Geometry2d::Point pos = Geometry2d::Point(),
                     float angle = 0, Time time = 0, int frame = 0)
        : pos(pos), angle(angle), time(time), source(-1), frameNumber(frame) {}

    Geometry2d::Point pos;
    float angle;  ///	in radians
    Time time;
    int source;
    int frameNumber;

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
    RobotFilter();

    /// Gives a new observation to the filter
    void update(const RobotObservation* obs);

    /// Generates a prediction of the ball's state at a given time in the
    /// future. This may clear robot->visible if the prediction is too long in
    /// the future to be reliable.
    void predict(Time time, RobotPose* robot);

private:
    static const int Num_Cameras = 4;

    /// Estimate for each camera
    RobotPose _estimate[Num_Cameras];

    /// Which camera we are using to track this robot, or -1 if it has not been
    /// seen recently
    int _camera;
};
