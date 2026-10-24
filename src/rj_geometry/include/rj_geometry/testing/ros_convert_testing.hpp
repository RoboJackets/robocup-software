#pragma once

#include <gtest/gtest.h>

#include <rclcpp/type_adapter.hpp>

template <typename RosType, typename CustomType>
void test_lossless_convert_ros_value(const RosType& ros_value) {
    CustomType custom{};
    rclcpp::TypeAdapter<CustomType, RosType>::convert_to_custom(ros_value, custom);
    RosType round_trip{};
    rclcpp::TypeAdapter<CustomType, RosType>::convert_to_ros_message(custom, round_trip);
    EXPECT_EQ(round_trip, ros_value);
}

template <typename CustomType, typename RosType>
void test_lossless_convert_cpp_value(const CustomType& cpp_value) {
    RosType ros_value{};
    rclcpp::TypeAdapter<CustomType, RosType>::convert_to_ros_message(cpp_value, ros_value);
    CustomType round_trip{};
    rclcpp::TypeAdapter<CustomType, RosType>::convert_to_custom(ros_value, round_trip);
    EXPECT_EQ(round_trip, cpp_value);
}
