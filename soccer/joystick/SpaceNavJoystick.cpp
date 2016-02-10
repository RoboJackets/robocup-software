#include "SpaceNavJoystick.hpp"
#include <Geometry2d/Util.hpp>
#include <spnav.h>
#include <iostream>

using namespace std;

REGISTER_CONFIGURABLE(SpaceNavJoystick)

ConfigDouble* SpaceNavJoystick::DribblerPositiveDeadzone;
ConfigDouble* SpaceNavJoystick::DribblerNegativeDeadzone;

SpaceNavJoystick::SpaceNavJoystick() {
    _daemonConnected = false;
    _daemonTried = false;
}

SpaceNavJoystick::~SpaceNavJoystick() { close(); }

bool SpaceNavJoystick::valid() const { return _daemonConnected; }

void SpaceNavJoystick::update() {
    QMutexLocker(&mutex());

    //  try again if we failed last time
    if (!_daemonConnected && !_daemonTried) open();

    //  abort
    if (!_daemonConnected) return;

    _controlValues.kick = false;
    _controlValues.chip = false;

    //  read events from spacenav.  There may be many of them. we're only
    //  interested in the latest motion event, but we want to take note of any
    //  button event that happens
    spnav_event sev;
    while (spnav_poll_event(&sev)) {
        if (sev.type == SPNAV_EVENT_MOTION) {
            //  note: spacenav axes range from -350 to 350

            //  we don't want to increment it at each iteration, otherwise it
            //  would change dribbler
            //  speed unusably quickly.  Instead we have a delay between each
            //  increment

            RJ::Time now = RJ::timestamp();
            const RJ::Time DribbleStepTime = 300000;
            if ((sev.motion.y >
                 SpaceNavJoystick::DribblerPositiveDeadzone->value()) ||
                (sev.motion.y <
                 SpaceNavJoystick::DribblerNegativeDeadzone->value())) {
                if ((now - _lastDribbleTime) > DribbleStepTime) {
                    _lastDribbleTime = now;

                    _controlValues.dribblerPower +=
                        0.1 * -sign<int>(sev.motion.y);
                    if (_controlValues.dribblerPower < 0)
                        _controlValues.dribblerPower = 0;
                    if (_controlValues.dribblerPower > 1)
                        _controlValues.dribblerPower = 1;
                }

                _controlValues.translation = Geometry2d::Point(0, 0);
                _controlValues.rotation = 0;
            }

            else {
                //  spacenav x is side-to-side and z is forward-to-backward
                //  strafe/translate
                //  translation starts out as a normalized value from (-1,-1) to
                //  (1,1)
                _controlValues.translation = Geometry2d::Point(
                    sev.motion.x / 350.0, sev.motion.z / 350.0);

                //  create a mouse deadzone - this keeps the robot from moving
                //  if
                //  the joystick is shifted just ever so slightly
                if (_controlValues.translation.mag() < 0.05) {
                    _controlValues.translation = Geometry2d::Point(0, 0);
                }

                //  twisting the spacenav corresponds to sev.motion.ry
                _controlValues.rotation = sev.motion.ry / 350.0;
                if (abs<float>(_controlValues.rotation) < 0.05) {
                    _controlValues.rotation = 0;
                }
            }

            _controlValues.dribble = _controlValues.dribblerPower > 0.01;

        } else {
            //  it's a button event!
            if (sev.button.press) {
                // spacenav button 0 is left, button 1 is right
                // we chose 0 == kicker, 1 == chipper
                if (sev.button.bnum == 1) {
                    _controlValues.chip = true;
                } else {
                    _controlValues.kick = true;
                }
            }
        }

        //  we can't control kick speed with the SpaceNavigator mouse, so
        //  default to half power
        _controlValues.kickPower = 0.5;
    }
}

void SpaceNavJoystick::createConfiguration(Configuration* cfg) {
    DribblerPositiveDeadzone = new ConfigDouble(
        cfg, "SpaceNavJoystick/Positive Dribbler Deadzone", 100);
    DribblerNegativeDeadzone = new ConfigDouble(
        cfg, "SpaceNavJoystick/Negative Dribbler Deadzone", -150);
}

JoystickControlValues SpaceNavJoystick::getJoystickControlValues() {
    return _controlValues;
}

void SpaceNavJoystick::reset() {
    if (_daemonConnected) {
        //  clear any old events in the queue
        spnav_remove_events(SPNAV_EVENT_ANY);
    }

    //  reset commands
    _controlValues = JoystickControlValues();
}

void SpaceNavJoystick::close() { spnav_close(); }

void SpaceNavJoystick::open() {
    _daemonConnected = spnav_open() != -1;
    _daemonTried = true;
    if (!_daemonConnected) {
        cerr << "Unable to connect to spacenav daemon.  Make sure spacenavd is "
                "running if you want to use a 3d mouse to drive" << endl;
    } else {
        cout << "Connected to spacenav daemon!  If a 3d mouse is connected, "
                "you can use it to drive" << endl;
    }
}
