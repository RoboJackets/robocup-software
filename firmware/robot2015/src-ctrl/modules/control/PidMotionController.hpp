#pragma once

#include <array>
#include "MotionController.hpp"
#include "Pid.hpp"

class PidMotionController : public MotionController {
public:
    void setTranslationalPidValues(float p, float i, float d) {}

    void setRotationalPidValues(float p, float i, float d) {}

    std::array<uint16_t, 4> run(std::array<float, 3> currVel) {
        // TODO: implement
        return {0, 0, 0, 0};
    }

private:
    Pid _xPid, _yPid, _wPid;
};
