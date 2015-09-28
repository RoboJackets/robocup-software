#include "Pid.hpp"

#include <cstring>
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;

Pid::Pid(float p, float i, float d, unsigned int windup) : _oldErr() {
    _windupLoc = 0;
    _errSum = 0;

    _lastErr = 0;

    kp = p;
    ki = i;
    kd = d;

    _windup = 0;
    setWindup(windup);
}

void Pid::setWindup(unsigned int w) {
    if (w != _windup) {
        _windup = w;

        if (w > 0) {
            _oldErr.resize(w);
            _errSum = 0;
            std::fill(_oldErr.begin(), _oldErr.end(), 0);
        } else {
            _oldErr.clear();
        }
    }
}

float Pid::run(const float err) {
    if (isnan(err)) {
        return 0;
    }
    float dErr = err - _lastErr;
    _lastErr = err;

    _errSum += err;

    if (_windup > 0) {
        _errSum -= _oldErr[_windupLoc];
        _oldErr[_windupLoc] = err;

        _windupLoc = (_windupLoc + 1) % _windup;
    }

    return (err * kp) + (_errSum * ki) + (dErr * kd);
}

void Pid::clearWindup() {
    _errSum = 0;
    std::fill(_oldErr.begin(), _oldErr.end(), 0);
}
