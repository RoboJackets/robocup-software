#pragma once

#include <ostream>

#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>

/**
 * \brief Stores the outputs published by MotionControl
 */
struct MotionSetpoint {
    using Msg = rj_msgs::msg::MotionSetpoint;

    double xvelocity = 0.;
    double yvelocity = 0.;
    double avelocity = 0.;

    MotionSetpoint() {}
    MotionSetpoint(double x, double y, double a) : xvelocity(x), yvelocity(y), avelocity(a) {}

    friend std::ostream& operator<<(std::ostream& stream, const MotionSetpoint& setpoint) {
        stream << "MotionSetpoint(" << setpoint.xvelocity << ", " << setpoint.yvelocity << ", "
               << setpoint.avelocity << ")";
        return stream;
    }
};

namespace rj_convert {

template <>
struct RosConverter<MotionSetpoint, MotionSetpoint::Msg> {
    static MotionSetpoint::Msg to_ros(const MotionSetpoint& from) {
        return rj_msgs::build<MotionSetpoint::Msg>()
            .velocity_x_mps(from.xvelocity)
            .velocity_y_mps(from.yvelocity)
            .velocity_z_radps(from.avelocity);
    }

    static MotionSetpoint from_ros(const MotionSetpoint::Msg& from) {
        return MotionSetpoint{from.velocity_x_mps, from.velocity_y_mps, from.velocity_z_radps};
    }
};

ASSOCIATE_CPP_ROS(MotionSetpoint, MotionSetpoint::Msg);

}  // namespace rj_convert