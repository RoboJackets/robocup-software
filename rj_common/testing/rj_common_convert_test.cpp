#include <gtest/gtest.h>

#include <rj_common/time.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

TEST(ROSConvert, time_lossless_convert) {
    TEST_LOSSLESS_CONVERT_ROS_VALUE(RJ::Time, rclcpp::Time,
                                    rclcpp::Time(123456));
    TEST_LOSSLESS_CONVERT_CPP_VALUE(RJ::Time, rclcpp::Time, RJ::now());
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    using Cvt = rj_convert::RosConverter<RJ::Seconds, rclcpp::Duration>;
    EXPECT_NEAR(Cvt::from_ros(Cvt::to_ros(RJ::Seconds(1.0))).count(), 1.0,
                1e-6);
    EXPECT_NEAR(
        Cvt::to_ros(Cvt::from_ros(rclcpp::Duration(12345))).nanoseconds(),
        12345, 1);
}
