#pragma once

#include <sys/time.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <rclcpp/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <string>

using namespace std::chrono_literals;

namespace RJ {

/// type for storing time in microseconds
using Time = std::chrono::system_clock::time_point;
using Timestamp = int64_t;  // Time in microseconds
using Seconds = std::chrono::duration<double>;
/** returns the local system timestamp in microseconds */

template <class Duration>
constexpr int64_t num_microseconds(Duration d) {
    return std::chrono::duration_cast<std::chrono::microseconds>(d).count();
}

inline Time now() {
    return std::chrono::system_clock::now();
    // struct timeval time;
    // gettimeofday(&time, nullptr);
    // return (Time)time.tv_sec * 1000000 + (Time)time.tv_usec;
}

constexpr Timestamp timestamp(Time time) {
    return num_microseconds(time.time_since_epoch());
}

inline Timestamp timestamp() { return timestamp(now()); }

/// Converts a decimal number of seconds to an integer timestamp in microseconds
constexpr RJ::Timestamp secs_to_timestamp(double secs) {
    return secs * 1000000.0f;
}

template <class Duration>
constexpr double num_seconds(Duration d) {
    return std::chrono::duration<double>(d).count();
}

/// Converts an integer timestamp in microseconds to a decimal number of seconds
constexpr float timestamp_to_secs(RJ::Timestamp timestamp) {
    return (float)(timestamp / 1000000.0f);
}

}  // namespace RJ

inline RJ::Time operator+(const RJ::Time& time, const RJ::Seconds& sec) {
    return time + std::chrono::duration_cast<RJ::Time::duration>(sec);
}

inline RJ::Time operator-(const RJ::Time& time, const RJ::Seconds& sec) {
    return time - std::chrono::duration_cast<RJ::Time::duration>(sec);
}

inline std::string to_string(RJ::Seconds seconds) {
    return std::to_string(seconds.count()) + "(Seconds)";
}

inline std::ostream& operator<<(std::ostream& os, RJ::Seconds seconds) {
    os << to_string(seconds);
    return os;
}

namespace rclcpp {

template <>
struct TypeAdapter<RJ::Time, rclcpp::Time> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Time;
    using ros_message_type = rclcpp::Time;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        const int64_t nanos =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                source.time_since_epoch())
                .count();
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

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        const auto time = rj_convert::convert_to_ros<RJ::Time, rclcpp::Time>(source);
        destination.sec = static_cast<int32_t>(time.seconds());
        destination.nanosec = static_cast<uint32_t>(time.nanoseconds() % 1000000000);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination =
            rj_convert::convert_from_ros<rclcpp::Time, RJ::Time>(
                rclcpp::Time{source.sec, source.nanosec});
    }
};
//std::chrono::duration_cast<std::chrono::nanoseconds>(value).count()
template <>
struct TypeAdapter<RJ::Seconds, rclcpp::Duration> {
    using is_specialized = std::true_type;
    using custom_type = RJ::Seconds;
    using ros_message_type = rclcpp::Duration;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(source));
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

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        const auto duration =
            rj_convert::convert_to_ros<RJ::Seconds, rclcpp::Duration>(source);
        destination.sec = static_cast<int32_t>(duration.seconds());
        destination.nanosec =
            static_cast<uint32_t>(duration.nanoseconds() % 1000000000);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination =
            rj_convert::convert_from_ros<builtin_interfaces::msg::Duration, RJ::Seconds>(
                source);
    }
};


}  // namespace rclcpp
