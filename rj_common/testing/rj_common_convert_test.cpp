#include <gtest/gtest.h>

#include <rj_common/time.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

TEST(ROSConvert, time_lossless_convert) {
    test_lossless_convert_ros_value(builtin_interfaces::msg::Time(rclcpp::Time(123456)));
    test_lossless_convert_cpp_value(RJ::now());
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    using Cvt = rj_convert::RosConverter<RJ::Seconds, rclcpp::Duration>;
    EXPECT_NEAR(Cvt::from_ros(Cvt::to_ros(RJ::Seconds(1.0))).count(), 1.0, 1e-6);
    EXPECT_NEAR(
        Cvt::to_ros(Cvt::from_ros(rclcpp::Duration(std::chrono::nanoseconds(12345)))).nanoseconds(),
        12345, 1);
}
