#include <gtest/gtest.h>

#include <rj_convert/ros_convert.hpp>

template <typename CppType, typename RosType>
void test_lossless_convert_ros_value(const RosType& ros_value) {
    EXPECT_EQ((rj_convert::convert_to_ros<CppType, RosType>(
                  rj_convert::convert_from_ros<CppType, RosType>(ros_value))),
              ros_value);
}

template <typename CppType, typename RosType>
void test_lossless_convert_cpp_value(const CppType& cpp_value) {
    EXPECT_EQ((rj_convert::convert_from_ros<CppType, RosType>(
                  rj_convert::convert_to_ros<CppType, RosType>(cpp_value))),
              cpp_value);
}
