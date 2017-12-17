
#include "Pid.hpp"

#include <cmath>

using namespace std;

Pid::Pid(float p, float i, float d, unsigned int windup, float dAlpha)
        : kp(p), ki(i), kd(d), derivAlpha(dAlpha), _windup(0), _windupLoc(0), _errSum(0), _lastError(0),
          _lastDeriv(0), _oldErr(), _saturated(false) {
    setWindup(windup);
}

void Pid::setWindup(unsigned int w) {
    if (w != _windup) {
        _windup = w;

        if (w > 0) {
            _oldErr.resize(w);
            _errSum = 0;
            _windupLoc = _windupLoc % _windup;
            std::fill(_oldErr.begin(), _oldErr.end(), 0);
        } else {
            _oldErr.clear();
        }
    }
}

float Pid::run(const float err) {
    if (isnan(err)) return 0;

    float integralErr;
    // integral
    if (!_saturated) {
        integralErr = err;
    } else {
        integralErr = 0;
    }

    _errSum += integralErr;

    if (_windup > 0) {
        _errSum -= _oldErr[_windupLoc];
        _oldErr[_windupLoc] = integralErr;

        _windupLoc = (_windupLoc + 1) % _windup;
    }

    // derivative (with alpha filter)
    float newDeriv = (err - _lastError); // compute newest derivative
    float derivative = derivAlpha * _lastDeriv + (1 - derivAlpha) * newDeriv;

    // update our state variables
    _lastError = err;
    _lastDeriv = derivative;
    if (_windup>0) {
        return (err * kp) + (_errSum * ki)/_windup + (derivative * kd);
    } else {
        return (err * kp) + (_errSum * ki) + (derivative * kd);
    }
}

void Pid::clearWindup() {
    _errSum = 0;
    std::fill(_oldErr.begin(), _oldErr.end(), 0);
}

void Pid::initializeTuner() {
    _tuner = make_unique<PidTuner>(kp,ki,kd);
}

void Pid::setFromTuner() {
    kp = _tuner->getP();
    ki = _tuner->getI();
    kd = _tuner->getD();
}

void Pid::startTunerCycle() {
    _tuner->startCycle();
    setFromTuner();
}

void Pid::runTuner() {
    _tuner->run(_lastError);
}

bool Pid::endTunerCycle() {
    if(_tuner->endCycle()) {
        return true;
    } else {
        setFromTuner();
        return false;
    }
}
