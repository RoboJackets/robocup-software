#include <gtest/gtest.h>

#include <rj_common/time.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

TEST(ROSConvert, time_lossless_convert) {
    test_lossless_convert_ros_value<builtin_interfaces::msg::Time, RJ::Time>(
        builtin_interfaces::msg::Time(rclcpp::Time(123456)));
    test_lossless_convert_cpp_value<RJ::Time, builtin_interfaces::msg::Time>(RJ::now());
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    rclcpp::Duration converted_duration(std::chrono::nanoseconds(0));
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_ros(
        RJ::Seconds(1.0), converted_duration);

    RJ::Seconds converted_seconds;
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_custom(
        converted_duration, converted_seconds);
    EXPECT_NEAR(converted_seconds.count(), 1.0, 1e-6);

    const auto source_duration = rclcpp::Duration(std::chrono::nanoseconds(12345));
    RJ::Seconds source_seconds;
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_custom(
        source_duration, source_seconds);
    rclcpp::Duration round_trip_duration(std::chrono::nanoseconds(0));
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_ros(
        source_seconds, round_trip_duration);
    EXPECT_NEAR(round_trip_duration.nanoseconds(), 12345, 1);
}
