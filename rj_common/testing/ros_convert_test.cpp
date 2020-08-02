#include "rj_common/ros_convert.hpp"

#include <gtest/gtest.h>

#include "rj_common/testing/ros_convert_testing.hpp"

TEST(ROSConvert, primitives_have_lossless_convert) {
    TEST_LOSSLESS_CONVERT_ROS_VALUE(int, int, 5);
    TEST_LOSSLESS_CONVERT_CPP_VALUE(int, int, 5);
}

TEST(ROSConvert, time_lossless_convert) {
    TEST_LOSSLESS_CONVERT_ROS_VALUE(RJ::Time, rclcpp::Time,
                                    rclcpp::Time(123456));
    TEST_LOSSLESS_CONVERT_CPP_VALUE(RJ::Time, rclcpp::Time, RJ::now());
}

TEST(ROSConvert, duration_lossless_convert) {
    // We don't expect perfect equality for seconds, because float comparisons.
    using Cvt = rj_common::RosConverter<RJ::Seconds, rclcpp::Duration>;
    EXPECT_NEAR(Cvt::from_ros(Cvt::to_ros(RJ::Seconds(1.0))).count(), 1.0,
                1e-6);
    EXPECT_NEAR(
        Cvt::to_ros(Cvt::from_ros(rclcpp::Duration(12345))).nanoseconds(),
        12345, 1);
}

TEST(ROSConvert, vector_primitive) {
    std::vector<int> vec{1, 2, 3, 4, 5};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<int>, std::vector<int>, vec);
    TEST_LOSSLESS_CONVERT_CPP_VALUE(std::vector<int>, std::vector<int>, vec);
}

TEST(ROSConvert, vector_ros_type) {
    std::vector<rclcpp::Time> vec_ros{rclcpp::Time(1), rclcpp::Time(2),
                                      rclcpp::Time(3)};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<RJ::Time>,
                                    std::vector<rclcpp::Time>, vec_ros);
}

TEST(ROSConvert, vector_nested) {
    std::vector<std::vector<rclcpp::Time>> vec_ros{
        {rclcpp::Time(1), rclcpp::Time(2)}, {rclcpp::Time(3)}};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<std::vector<RJ::Time>>,
                                    std::vector<std::vector<rclcpp::Time>>,
                                    vec_ros);
}

TEST(ROSConvert, array_primitive) {
    std::array<int, 5> vec{1, 2, 3, 4, 5};

    // Type has a comma in it, so we have to rename it for macros to work
    using array_int = std::array<int, 5>;
    TEST_LOSSLESS_CONVERT_ROS_VALUE(array_int, array_int, vec);
    TEST_LOSSLESS_CONVERT_CPP_VALUE(array_int, array_int, vec);
}
