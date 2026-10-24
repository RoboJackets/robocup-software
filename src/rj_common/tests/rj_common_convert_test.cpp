#include <gtest/gtest.h>

#include <rj_common/time.hpp>
#include <rj_geometry/testing/ros_convert_testing.hpp>


TEST(ROSConvert, time_lossless_convert) {
    test_lossless_convert_ros_value<builtin_interfaces::msg::Time, RJ::Time>(
        builtin_interfaces::msg::Time(rclcpp::Time(123456)));
    test_lossless_convert_cpp_value<RJ::Time, builtin_interfaces::msg::Time>(RJ::now());
}

TEST(ROSConvert, time_near_integer_second_keeps_nanoseconds) {
    // seconds() rounds this epoch-scale value up to the next whole second.
    constexpr int32_t kSeconds = 1700000000;
    constexpr uint32_t kNanoseconds = 999999999;
    const RJ::Time source{std::chrono::seconds(kSeconds) + std::chrono::nanoseconds(kNanoseconds)};

    builtin_interfaces::msg::Time message;
    rclcpp::TypeAdapter<RJ::Time, builtin_interfaces::msg::Time>::convert_to_ros_message(source,
                                                                                         message);
    EXPECT_EQ(message.sec, kSeconds);
    EXPECT_EQ(message.nanosec, kNanoseconds);
    test_lossless_convert_cpp_value<RJ::Time, builtin_interfaces::msg::Time>(source);
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    rclcpp::Duration converted_duration(std::chrono::nanoseconds(0));
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_ros_message(RJ::Seconds(1.0),
                                                                               converted_duration);

    RJ::Seconds converted_seconds;
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_custom(converted_duration,
                                                                          converted_seconds);
    EXPECT_NEAR(converted_seconds.count(), 1.0, 1e-6);

    const auto source_duration = rclcpp::Duration(std::chrono::nanoseconds(12345));
    RJ::Seconds source_seconds;
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_custom(source_duration,
                                                                          source_seconds);
    rclcpp::Duration round_trip_duration(std::chrono::nanoseconds(0));
    rclcpp::TypeAdapter<RJ::Seconds, rclcpp::Duration>::convert_to_ros_message(source_seconds,
                                                                               round_trip_duration);
    EXPECT_NEAR(round_trip_duration.nanoseconds(), 12345, 1);
}

TEST(ROSConvert, builtin_duration_lossless_convert) {
    builtin_interfaces::msg::Duration ros_duration;
    ros_duration.sec = 12;
    ros_duration.nanosec = 345678901;

    test_lossless_convert_ros_value<builtin_interfaces::msg::Duration, RJ::Seconds>(ros_duration);
    test_lossless_convert_cpp_value<RJ::Seconds, builtin_interfaces::msg::Duration>(
        RJ::Seconds(12.345678901));
}

TEST(ROSConvert, builtin_duration_negative_keeps_nanoseconds) {
    // A negative remainder must stay in nanosec, with the sign carried by sec.
    const RJ::Seconds source{std::chrono::nanoseconds(-1500000001)};

    builtin_interfaces::msg::Duration message;
    rclcpp::TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration>::convert_to_ros_message(
        source, message);

    const auto expected = static_cast<builtin_interfaces::msg::Duration>(
        rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(source)));
    EXPECT_EQ(message, expected);
    EXPECT_EQ(message.sec, -2);
    EXPECT_EQ(message.nanosec, 499999999u);
    test_lossless_convert_cpp_value<RJ::Seconds, builtin_interfaces::msg::Duration>(source);
}
