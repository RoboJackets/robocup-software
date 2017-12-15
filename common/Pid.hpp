#pragma once
#include <vector>
#include <memory>
#include "pidTuner.hpp"

class Pid {
public:
    Pid(float p = 0, float i = 0, float d = 0, unsigned int windup = 0, float dAlpha = 0);

    float run(float err);

    unsigned int windup() const { return _windup; }

    void setWindup(unsigned int w);

    /** clear any windup */
    void clearWindup();


    //Pid Tuning Functions
    void initializeTuner();
    void startTunerCycle();
    void runTuner();
    bool endTunerCycle();


    float kp, ki, kd;
    float derivAlpha;

    void set_saturated(bool is_saturated) { _saturated = is_saturated; }

private:

    /** amount to sum up */
    unsigned int _windup;

    unsigned int _windupLoc;

    bool _saturated;

    float _errSum;

    float _lastError;

    float _lastDeriv;

    std::vector<float> _oldErr{};

    std::unique_ptr <PidTuner> _tuner;
    void setFromTuner();
};
