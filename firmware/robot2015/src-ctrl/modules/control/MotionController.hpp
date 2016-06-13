#pragma once

#include <array>
#include "RtosTimerHelper.hpp"
#include "RobotModel.hpp"

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
        commandTimeout(); // reset
        _commandTimeout.start(COMMAND_TIMEOUT_INTERVAL);
    }

    /** Set target velocity. This is typically called whenever a new radio
     * packet is received that contains velocity commands.
     * */
    virtual void setTargetVel(Eigen::Vector3f target) {
        // reset timeout timer
        _commandTimeout.start(COMMAND_TIMEOUT_INTERVAL);

        _targetVel = target;
    }

    /** Run the controller and return duty cycle values for each of the 4 drive
     * motors.
     */
    virtual std::array<int16_t, 4> run(
        const std::array<int16_t, 4>& encoderDeltas, float dt) = 0;

protected:
    /* This is called when the timeout timer fires, indicating that the target
     * velocity hasn't been set for a while.  It resets the target velocity to
     * zero.
     */
    void commandTimeout() { _targetVel = {0, 0, 0}; }

    // The current target velocity in [x, y, w] format.
    Eigen::Vector3f _targetVel;

private:
    RtosTimerHelper _commandTimeout;
};
