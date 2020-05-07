
#pragma once

#include <Robot.hpp>
#include <protobuf/RadioTx.pb.h>

#include <QMutex>
#include <boost/utility.hpp>
#include <stdexcept>
#include <stdint.h>
#include <vector>
#include <SDL.h>

struct InputDeviceControlValues {
public:
    InputDeviceControlValues()
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
 * to return a InputDeviceControlValues object indicating the desired driving
 */
class InputDevice : boost::noncopyable {
public:
    InputDevice() : _mutex(QMutex::Recursive){};
    virtual ~InputDevice(){};

    // virtual void initDeviceType();

    /**
     * @brief Whether or not this InputDevice is connected to a real device
     */
    virtual bool valid() const = 0;

    /**
     * @brief Resets any stored values
     * @details InputDevices are reset whenever the robot they're controlling is
     * switched
     */
    virtual void reset() = 0;

    /**
     * @brief Instructs the joystick to updates its values
     */
    virtual void update(SDL_Event& event) = 0;

    /**
     * @brief Returns the control values from this joystick
     * @details Note: The controls should be scaled between 0 and 1.
     * The processor handles scaling to real world values.
     * @return The control values for this joystick
     */
    virtual InputDeviceControlValues getInputDeviceControlValues() = 0;

    static void createConfiguration(Configuration* cfg);

    int getRobotId() { return robotId; }
    void setRobotId(int rId) { robotId = rId; }

    // declaration of Config values
    static ConfigDouble* InputDeviceRotationMaxSpeed;
    static ConfigDouble* InputDeviceRotationMaxDampedSpeed;
    static ConfigDouble* InputDeviceTranslationMaxSpeed;
    static ConfigDouble* InputDeviceTranslationMaxDampedSpeed;

protected:
    QMutex& mutex() { return _mutex; }
    int robotId;

private:
    QMutex _mutex;
};
