#pragma once

#include <sys/time.h>

#include <chrono>
#include <iostream>
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
constexpr int64_t numMicroseconds(Duration d) {
    return std::chrono::duration_cast<std::chrono::microseconds>(d).count();
}

inline Time now() {
    return std::chrono::system_clock::now();
    // struct timeval time;
    // gettimeofday(&time, nullptr);
    // return (Time)time.tv_sec * 1000000 + (Time)time.tv_usec;
}

constexpr Timestamp timestamp(Time time) {
    return numMicroseconds(time.time_since_epoch());
}

inline Timestamp timestamp() { return timestamp(now()); }

inline rclcpp::Time ToROSTime(RJ::Time time) {
    const int64_t nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              time.time_since_epoch())
                              .count();

    return rclcpp::Time{nanos};
}

inline RJ::Time FromROSTime(const rclcpp::Time& time) {
    const std::chrono::nanoseconds dur(time.nanoseconds());
    return RJ::Time{dur};
}

inline rclcpp::Duration ToROSDuration(RJ::Seconds seconds) {
    return rclcpp::Duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(seconds).count());
}

inline RJ::Seconds FromROSDuration(const rclcpp::Duration& duration) {
    const std::chrono::nanoseconds dur(duration.nanoseconds());
    return std::chrono::duration_cast<RJ::Seconds>(dur);
}

/// Converts a decimal number of seconds to an integer timestamp in microseconds
constexpr RJ::Timestamp SecsToTimestamp(double secs) {
    return secs * 1000000.0f;
}

template <class Duration>
constexpr double numSeconds(Duration d) {
    return std::chrono::duration<double>(d).count();
}

/// Converts an integer timestamp in microseconds to a decimal number of seconds
constexpr float TimestampToSecs(RJ::Timestamp timestamp) {
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
struct RosConverter<RJ::Seconds, rclcpp::Duration> {
    static rclcpp::Duration to_ros(const RJ::Seconds& value) {
        return rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(value)
                .count());
    }
    static RJ::Seconds from_ros(const rclcpp::Duration& value) {
        const std::chrono::nanoseconds dur(value.nanoseconds());
        return std::chrono::duration_cast<RJ::Seconds>(dur);
    }
};

} // namespace rj_convert