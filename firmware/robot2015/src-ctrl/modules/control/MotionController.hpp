#pragma once

#include <array>
#include "RtosTimerHelper.hpp"

/** Abstract superclass for a robot motion controller.  Its main job is to turn
 * velocity commands into duty cycle values to be sent to the fpga.
 */
class MotionController {
public:
    /** If this amount of time (in ms) elapses without setTargetVel() being
     * called, the target velocity is reset to zero.  This is a safety feature
     * to prevent robots from doing unwanted things when they lose radio
     * communication.
     */
    static const uint32_t COMMAND_TIMEOUT_INTERVAL = 250;

    MotionController()
        : _commandTimeout(this, &MotionController::commandTimeout,
                          osTimerOnce) {
        _commandTimeout.start(COMMAND_TIMEOUT_INTERVAL);
    }

    /** Set target velocity. This is typically called whenever a new radio
     * packet is received that contains velocity commands.
     * */
    virtual void setTargetVel(int16_t x, int16_t y, int16_t w) {
        // reset timeout timer
        _commandTimeout.start(COMMAND_TIMEOUT_INTERVAL);

        _targetVel = {x, y, w};
    }

    /** Run the controller and return duty cycle values for each of the 4 drive
     * motors given the robot's estimated current velocity.
     */
    virtual std::array<uint16_t, 4> run(std::array<float, 3> currVel) = 0;

protected:
    /* This is called when the timeout timer fires, indicating that the target
     * velocity hasn't been set for a while.  It resets the target velocity to
     * zero.
     */
    void commandTimeout() { _targetVel = {0, 0, 0}; }

private:
    // The current target velocity in [x, y, w] format.
    std::array<int16_t, 3> _targetVel;

    RtosTimerHelper _commandTimeout;
};
