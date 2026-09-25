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
    const auto converted_duration =
        rj_convert::convert_to_ros<RJ::Seconds, rclcpp::Duration>(RJ::Seconds(1.0));
    EXPECT_NEAR(
        rj_convert::convert_from_ros<rclcpp::Duration, RJ::Seconds>(converted_duration).count(),
        1.0, 1e-6);

    const auto source_duration = rclcpp::Duration(std::chrono::nanoseconds(12345));
    EXPECT_NEAR(
        rj_convert::convert_to_ros<RJ::Seconds, rclcpp::Duration>(
            rj_convert::convert_from_ros<rclcpp::Duration, RJ::Seconds>(source_duration))
            .nanoseconds(),
        12345, 1);
}
