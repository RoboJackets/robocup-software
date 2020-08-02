#include <rj_convert/ros_convert.hpp>

#include <gtest/gtest.h>

#include <rj_convert/testing/ros_convert_testing.hpp>

struct MockTime {
    rclcpp::Time time;
};

namespace rj_convert {

template<> struct RosConverter<MockTime, rclcpp::Time> {
    static rclcpp::Time to_ros(const MockTime& from) {
        return from.time;
    }

    static MockTime from_ros(const rclcpp::Time& from) {
        return MockTime{from};
    }
};

} // namespace rj_convert

TEST(ROSConvert, primitives_have_lossless_convert) {
    TEST_LOSSLESS_CONVERT_ROS_VALUE(int, int, 5);
    TEST_LOSSLESS_CONVERT_CPP_VALUE(int, int, 5);
}

TEST(ROSConvert, vector_primitive) {
    std::vector<int> vec{1, 2, 3, 4, 5};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<int>, std::vector<int>, vec);
    TEST_LOSSLESS_CONVERT_CPP_VALUE(std::vector<int>, std::vector<int>, vec);
}

TEST(ROSConvert, vector_ros_type) {
    std::vector<rclcpp::Time> vec_ros{rclcpp::Time(1), rclcpp::Time(2),
                                      rclcpp::Time(3)};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<MockTime>,
                                    std::vector<rclcpp::Time>, vec_ros);
}

TEST(ROSConvert, vector_nested) {
    std::vector<std::vector<rclcpp::Time>> vec_ros{
        {rclcpp::Time(1), rclcpp::Time(2)}, {rclcpp::Time(3)}};
    TEST_LOSSLESS_CONVERT_ROS_VALUE(std::vector<std::vector<MockTime>>,
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
