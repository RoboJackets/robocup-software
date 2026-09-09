#include <gtest/gtest.h>

#include <rj_common/time.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

TEST(ROSConvert, time_lossless_convert) {
    builtin_interfaces::msg::Time time;
    time.nanosec = 123456;
    test_lossless_convert_ros_value(time);
    test_lossless_convert_cpp_value(RJ::now());
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    using Cvt =
        rj_convert::RosConverter<RJ::Seconds, builtin_interfaces::msg::Duration>;
    EXPECT_NEAR(Cvt::from_ros(Cvt::to_ros(RJ::Seconds(1.0))).count(), 1.0, 1e-6);
    builtin_interfaces::msg::Duration input;
    input.nanosec = 12345;
    const auto duration = Cvt::to_ros(Cvt::from_ros(input));
    EXPECT_EQ(duration.sec, 0);
    EXPECT_EQ(duration.nanosec, 12345);
}
