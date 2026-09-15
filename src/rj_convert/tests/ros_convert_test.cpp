#include <builtin_interfaces/msg/time.hpp>
#include <gtest/gtest.h>

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

    static MockTime from_ros(const MockTime::Msg& from) { return MockTime{from}; }
};

ASSOCIATE_CPP_ROS(MockTime, MockTime::Msg);

}  // namespace rj_convert

TEST(ROSConvert, vector_ros_type) {
    MockTime::Msg first;
    first.sec = 1;
    MockTime::Msg second;
    second.sec = 2;
    MockTime::Msg third;
    third.sec = 3;
    std::vector<MockTime::Msg> vec_ros{first, second, third};
    test_lossless_convert_ros_value(vec_ros);
}

TEST(ROSConvert, vector_nested) {
    MockTime::Msg first;
    first.sec = 1;
    MockTime::Msg second;
    second.sec = 2;
    MockTime::Msg third;
    third.sec = 3;
    std::vector<std::vector<MockTime::Msg>> vec_ros{{first, second}, {third}};
    test_lossless_convert_ros_value(vec_ros);
}
