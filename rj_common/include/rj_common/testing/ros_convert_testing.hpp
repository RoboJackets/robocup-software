#include <gtest/gtest.h>

#include "rj_common/ros_convert.hpp"

#define TEST_LOSSLESS_CONVERT_ROS_VALUE(CppType, RosType, ros_value)           \
    {                                                                          \
        const auto& tmp_value = ros_value;                                     \
        EXPECT_EQ(                                                             \
            (rj_common::convert_to_ros<CppType, RosType>(rj_common::convert_from_ros<CppType, RosType>(tmp_value))), \
            tmp_value);                                                        \
    }

#define TEST_LOSSLESS_CONVERT_CPP_VALUE(CppType, RosType, cpp_value)           \
    {                                                                          \
        const auto& tmp_value = cpp_value;                                     \
        EXPECT_EQ(                                                             \
            (rj_common::convert_from_ros<CppType, RosType>(rj_common::convert_to_ros<CppType, RosType>(tmp_value))), \
            tmp_value);                                                        \
    }
