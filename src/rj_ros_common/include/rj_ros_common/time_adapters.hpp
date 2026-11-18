#pragma once

#include <chrono>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/type_adapter.hpp>

#include <rj_time/time.hpp>

namespace rclcpp {

template <>
struct TypeAdapter<RJ::Time, rclcpp::Time> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Time;
    using ros_message_type = rclcpp::Time;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        const int64_t nanos =
            std::chrono::duration_cast<std::chrono::nanoseconds>(source.time_since_epoch()).count();
        destination = rclcpp::Time{nanos};
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        const std::chrono::nanoseconds dur(source.nanoseconds());
        destination = RJ::Time{dur};
    }
};

template <>
struct TypeAdapter<RJ::Time, builtin_interfaces::msg::Time> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Time;
    using ros_message_type = builtin_interfaces::msg::Time;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::Time ros_time;
        rclcpp::TypeAdapter<RJ::Time, rclcpp::Time>::convert_to_ros_message(source, ros_time);
        destination = static_cast<builtin_interfaces::msg::Time>(ros_time);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<RJ::Time, rclcpp::Time>::convert_to_custom(
            rclcpp::Time{source.sec, source.nanosec}, destination);
    }
};
template <>
struct TypeAdapter<RJ::Seconds, rclcpp::Duration> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Seconds;
    using ros_message_type = rclcpp::Duration;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination =
            rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(source));
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        const std::chrono::nanoseconds dur(source.nanoseconds());
        destination = std::chrono::duration_cast<RJ::Seconds>(dur);
    }
};

template <>
struct TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Seconds;
    using ros_message_type = builtin_interfaces::msg::Duration;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination = static_cast<builtin_interfaces::msg::Duration>(
            rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(source)));
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        const auto duration =
            std::chrono::seconds(source.sec) + std::chrono::nanoseconds(source.nanosec);
        destination = std::chrono::duration_cast<RJ::Seconds>(duration);
    }
};

}  // namespace rclcpp
