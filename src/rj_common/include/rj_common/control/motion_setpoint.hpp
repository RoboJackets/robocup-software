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

    MotionSetpoint() = default;
    MotionSetpoint(double x, double y, double a) : xvelocity(x), yvelocity(y), avelocity(a) {}

    friend std::ostream& operator<<(std::ostream& stream, const MotionSetpoint& setpoint) {
        stream << "MotionSetpoint(" << setpoint.xvelocity << ", " << setpoint.yvelocity << ", "
               << setpoint.avelocity << ")";
        return stream;
    }
};

namespace rclcpp {

template <>
struct TypeAdapter<MotionSetpoint, MotionSetpoint::Msg> {
    using is_specialized = std::true_type;
    using custom_type = MotionSetpoint;
    using ros_message_type = MotionSetpoint::Msg;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = rj_msgs::build<MotionSetpoint::Msg>()
                          .velocity_x_mps(source.xvelocity)
                          .velocity_y_mps(source.yvelocity)
                          .velocity_z_radps(source.avelocity);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = MotionSetpoint{source.velocity_x_mps, source.velocity_y_mps,
                                     source.velocity_z_radps};
    }
};


}  // namespace rclcpp