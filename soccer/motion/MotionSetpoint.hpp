#pragma once
#include <ostream>

/**
 * \brief Stores the outputs published by MotionControl
 */
struct MotionSetpoint {
    double xvelocity = 0.;
    double yvelocity = 0.;
    double avelocity = 0.;
    void clear() { xvelocity = yvelocity = avelocity = 0; }
    MotionSetpoint() { clear(); }

    friend std::ostream& operator<<(std::ostream& stream,
                                    const MotionSetpoint& setpoint) {
        stream << "MotionSetpoint(" << setpoint.xvelocity << ", "
               << setpoint.yvelocity << ", " << setpoint.avelocity << ")";
        return stream;
    }
};
