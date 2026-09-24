#include <builtin_interfaces/msg/time.hpp>
#include <gtest/gtest.h>
#include <rclcpp/time.hpp>

#include <rj_convert/ros_convert.hpp>
#include <rj_convert/testing/ros_convert_testing.hpp>

struct MockTime {
    using Msg = builtin_interfaces::msg::Time;
    Msg time;

    bool operator==(const MockTime& other) const { return time == other.time; }
};

namespace rclcpp {

template <>
struct TypeAdapter<MockTime, MockTime::Msg> {
    using is_specialized = std::true_type;
    using custom_type = MockTime;
    using ros_message_type = MockTime::Msg;

    static void convert_to_ros(const custom_type& from, ros_message_type& to) {
        to = from.time;
    }

    static void convert_to_custom(const ros_message_type& from, custom_type& to) {
        to = MockTime{from};
    }
};

}  // namespace rclcpp

TEST(ROSConvert, primitives_have_lossless_convert) {
    test_lossless_convert_ros_value<int, int>(5);
    test_lossless_convert_cpp_value<int, int>(5);
}

TEST(ROSConvert, type_adapter_matches_rep_2007_contract) {
    using Adapter = rclcpp::adapt_type<MockTime>::as<MockTime::Msg>;
    static_assert(Adapter::is_specialized::value);

    MockTime custom{rclcpp::Time(42)};
    MockTime::Msg ros;
    Adapter::convert_to_ros(custom, ros);
    EXPECT_EQ(ros, custom.time);

    MockTime::Msg ros_for_rclcpp;
    Adapter::convert_to_ros(custom, ros_for_rclcpp);
    EXPECT_EQ(ros_for_rclcpp, custom.time);

    MockTime round_trip;
    Adapter::convert_to_custom(ros, round_trip);
    EXPECT_EQ(round_trip.time, custom.time);
}

TEST(ROSConvert, vector_primitive) {
    std::vector<int> vec{1, 2, 3, 4, 5};
    test_lossless_convert_ros_value<std::vector<int>, std::vector<int>>(vec);
    test_lossless_convert_cpp_value<std::vector<int>, std::vector<int>>(vec);
}

TEST(ROSConvert, vector_ros_type) {
    std::vector<MockTime::Msg> vec_ros{rclcpp::Time(1), rclcpp::Time(2), rclcpp::Time(3)};
    test_lossless_convert_ros_value<std::vector<MockTime::Msg>, std::vector<MockTime>>(
        vec_ros);
}

TEST(ROSConvert, vector_custom_type) {
    std::vector<MockTime> vec_custom{{rclcpp::Time(1)}, {rclcpp::Time(2)}};
    test_lossless_convert_cpp_value<std::vector<MockTime>, std::vector<MockTime::Msg>>(
        vec_custom);
}

TEST(ROSConvert, vector_nested) {
    std::vector<std::vector<MockTime::Msg>> vec_ros{{rclcpp::Time(1), rclcpp::Time(2)},
                                                    {rclcpp::Time(3)}};
    test_lossless_convert_ros_value<std::vector<std::vector<MockTime::Msg>>,
                                    std::vector<std::vector<MockTime>>>(
        vec_ros);
}