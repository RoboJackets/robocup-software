#pragma once

#include "Joystick.hpp"

/**
 * @brief Joystick class for using a SpaceNavigator 3d mouse for controlling the
 * robots
 * @details Control mappings:
 * * Translation - just as you'd expect.  Moving the know forward and sideways
 * corresponds directly with robot movement
 * * Rotation - again as you'd expect.  Rotating the knob rotates the robot
 * * Kick - left mouse button
 * * Chip - right mouse button
 * * Dribbler - push down on the knob to increase speed, pull up on it to
 * decrease speed
 *
 * Note: usage of this joystick requires the spacenavd userspace driver daemon
 * to be running.
 */
class SpaceNavJoystick : public Joystick {
public:
    SpaceNavJoystick();
    ~SpaceNavJoystick();

    /**
     * @brief Returns whether or not we can connect to the spacenav driver
     * @details Unfortunately there is no api available for detecting whether or
     * not
     * there is a SpaceNavigator connected - we can only tell whether or not we
     * can connect to the daemon.
     * This is less than ideal, but it's what we've got.
     * @return A boolean indicating if the joystick is valid
     */
    bool valid() const override;

    void reset() override;
    void update() override;

    JoystickControlValues getJoystickControlValues() override;

protected:
    void open();
    void close();

    bool _daemonConnected;
    bool _daemonTried;

    //  store normalized values
    JoystickControlValues _controlValues;

    RJ::Time _lastDribbleTime;
};
