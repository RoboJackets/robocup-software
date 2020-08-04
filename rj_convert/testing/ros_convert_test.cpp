#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

struct MockTime {
    using Msg = builtin_interfaces::msg::Time;
    Msg time;
};

namespace rj_convert {

template <>
struct RosConverter<MockTime, MockTime::Msg> {
    static MockTime::Msg to_ros(const MockTime& from) { return from.time; }

    static MockTime from_ros(const MockTime::Msg& from) {
        return MockTime{from};
    }
};

}  // namespace rj_convert

TEST(ROSConvert, primitives_have_lossless_convert) {
    test_lossless_convert_ros_value<int>(5);
    test_lossless_convert_cpp_value<int>(5);
}

TEST(ROSConvert, vector_primitive) {
    std::vector<int> vec{1, 2, 3, 4, 5};
    test_lossless_convert_ros_value<std::vector<int>>(vec);
    test_lossless_convert_cpp_value<std::vector<int>>(vec);
}

TEST(ROSConvert, vector_ros_type) {
    std::vector<MockTime::Msg> vec_ros{rclcpp::Time(1), rclcpp::Time(2),
                                       rclcpp::Time(3)};
    test_lossless_convert_ros_value<std::vector<MockTime>>(vec_ros);
}

TEST(ROSConvert, vector_nested) {
    std::vector<std::vector<MockTime::Msg>> vec_ros{
        {rclcpp::Time(1), rclcpp::Time(2)}, {rclcpp::Time(3)}};
    test_lossless_convert_ros_value<std::vector<std::vector<MockTime>>>(
        vec_ros);
}

TEST(ROSConvert, array_primitive) {
    std::array<int, 5> vec{1, 2, 3, 4, 5};

    using array_int = std::array<int, 5>;
    test_lossless_convert_ros_value<array_int>(vec);
    test_lossless_convert_cpp_value<array_int>(vec);
}
