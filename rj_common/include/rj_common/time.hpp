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

namespace rj_convert {

template <>
struct RosConverter<RJ::Time, rclcpp::Time> {
    static rclcpp::Time to_ros(const RJ::Time& value) {
        const int64_t nanos =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                value.time_since_epoch())
                .count();

        return rclcpp::Time{nanos};
    }
    static RJ::Time from_ros(const rclcpp::Time& value) {
        const std::chrono::nanoseconds dur(value.nanoseconds());
        return RJ::Time{dur};
    }
};

template <>
struct RosConverter<RJ::Time, builtin_interfaces::msg::Time> {
    static rclcpp::Time to_ros(const RJ::Time& value) {
        return RosConverter<RJ::Time, rclcpp::Time>::to_ros(value);
    }
    static RJ::Time from_ros(const rclcpp::Time& value) {
        return RosConverter<RJ::Time, rclcpp::Time>::from_ros(value);
    }
};
//std::chrono::duration_cast<std::chrono::nanoseconds>(value).count()
template <>
struct RosConverter<RJ::Seconds, rclcpp::Duration> {
    static rclcpp::Duration to_ros(const RJ::Seconds& value) {
        return rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(value));
    }
    static RJ::Seconds from_ros(const rclcpp::Duration& value) {
        const std::chrono::nanoseconds dur(value.nanoseconds());
        return std::chrono::duration_cast<RJ::Seconds>(dur);
    }
};

ASSOCIATE_CPP_ROS(RJ::Time, builtin_interfaces::msg::Time);

template <>
struct RosConverter<RJ::Seconds, builtin_interfaces::msg::Duration> {
    static rclcpp::Duration to_ros(const RJ::Seconds& value) {
        return RosConverter<RJ::Seconds, rclcpp::Duration>::to_ros(value);
    }
    static RJ::Seconds from_ros(const rclcpp::Duration& value) {
        return RosConverter<RJ::Seconds, rclcpp::Duration>::from_ros(value);
    }
};

ASSOCIATE_CPP_ROS(RJ::Seconds, builtin_interfaces::msg::Duration);

}  // namespace rj_convert
