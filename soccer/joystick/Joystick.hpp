
#pragma once

#include <protobuf/RadioTx.pb.h>

#include <Robot.hpp>
#include <boost/utility.hpp>
#include <mutex>
#include <stdexcept>
#include <vector>

struct JoystickControlValues {
public:
    JoystickControlValues()
        : translation(0, 0),
          rotation(0),
          kickPower(0),
          dribblerPower(0),
          kick(false),
          chip(false),
          dribble(false),
          kickOnBreak(false) {}

    Geometry2d::Point translation;
    double rotation;
    double kickPower, dribblerPower;
    bool kick, chip, dribble, kickOnBreak;
};

/**
 * @brief Used for manually controlling robots
 * @details The joystick's main jobs are to report its status (valid or not) and
 * to return a JoystickControlValues object indicating the desired driving
 */
class Joystick : boost::noncopyable {
public:
    virtual ~Joystick() = default;

    /**
     * @brief Whether or not this Joystick is connected to a real device
     */
    virtual bool valid() const = 0;

    /**
     * @brief Resets any stored values
     * @details Joysticks are reset whenever the robot they're controlling is
     * switched
     */
    virtual void reset() = 0;

    /**
     * @brief Instructs the joystick to updates its values
     */
    virtual void update() = 0;

    /**
     * @brief Returns the control values from this joystick
     * @details Note: The controls should be scaled between 0 and 1.
     * The processor handles scaling to real world values.
     * @return The control values for this joystick
     */
    virtual JoystickControlValues getJoystickControlValues() = 0;

    static void createConfiguration(Configuration* cfg);

    int getRobotId() { return robotId; }
    void setRobotId(int rId) { robotId = rId; }

    // declaration of Config values
    static ConfigDouble* JoystickRotationMaxSpeed;
    static ConfigDouble* JoystickRotationMaxDampedSpeed;
    static ConfigDouble* JoystickTranslationMaxSpeed;
    static ConfigDouble* JoystickTranslationMaxDampedSpeed;

protected:
    std::lock_guard<std::mutex> lock_mutex() { return std::lock_guard(_mutex); }
    int robotId;

private:
    std::mutex _mutex;
};
