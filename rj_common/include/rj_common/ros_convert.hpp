#pragma once

#include <rclcpp/time.hpp>
#include <type_traits>

#include "rj_common/time.hpp"

template <typename CppType, typename RosType>
struct RosConverter {};

#define CONVERT_PRIMITIVE(type)                                   \
    template <>                                                   \
    struct RosConverter<type, type> {                             \
        static type to_ros(const type& value) { return value; }   \
        static type from_ros(const type& value) { return value; } \
    }

CONVERT_PRIMITIVE(int8_t);
CONVERT_PRIMITIVE(int16_t);
CONVERT_PRIMITIVE(int32_t);
CONVERT_PRIMITIVE(int64_t);

CONVERT_PRIMITIVE(uint8_t);
CONVERT_PRIMITIVE(uint16_t);
CONVERT_PRIMITIVE(uint32_t);
CONVERT_PRIMITIVE(uint64_t);

CONVERT_PRIMITIVE(float);
CONVERT_PRIMITIVE(double);

CONVERT_PRIMITIVE(bool);
CONVERT_PRIMITIVE(std::string);
CONVERT_PRIMITIVE(std::u16string);

#undef CONVERT_PRIMITIVE

template <>
struct RosConverter<RJ::Time, rclcpp::Time> {
    static rclcpp::Time to_ros(const RJ::Time& value) {
        const int64_t nanos =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                value.time_since_epoch())
                .count();

        return rclcpp::Time{nanos};
    }
    static RJ::Time from_ros(const rclcpp::Time& value) {
        const std::chrono::nanoseconds dur(value.nanoseconds());
        return RJ::Time{dur};
    }
};

template <>
struct RosConverter<RJ::Seconds, rclcpp::Duration> {
    static rclcpp::Duration to_ros(const RJ::Seconds& value) {
        return rclcpp::Duration(
            std::chrono::duration_cast<std::chrono::nanoseconds>(value)
                .count());
    }
    static RJ::Seconds from_ros(const rclcpp::Duration& value) {
        const std::chrono::nanoseconds dur(value.nanoseconds());
        return std::chrono::duration_cast<RJ::Seconds>(dur);
    }
};

template <typename CppItem, typename RosItem>
struct RosConverter<std::vector<CppItem>, std::vector<RosItem>> {
    static std::vector<RosItem> to_ros(const std::vector<CppItem>& value) {
        std::vector<RosItem> result;
        result.reserve(value.size());
        std::transform(std::begin(value), std::end(value),
                       std::back_inserter(result),
                       RosConverter<CppItem, RosItem>::to_ros);
        return result;
    }

    static std::vector<CppItem> from_ros(const std::vector<RosItem>& value) {
        std::vector<CppItem> result;
        result.reserve(value.size());
        std::transform(std::begin(value), std::end(value),
                       std::back_inserter(result),
                       RosConverter<CppItem, RosItem>::from_ros);
        return result;
    }
};

template <typename CppItem, typename RosItem, size_t size>
struct RosConverter<std::array<CppItem, size>, std::array<RosItem, size>> {
    static std::array<RosItem, size> to_ros(
        const std::array<CppItem, size>& value) {
        std::array<RosItem, size> result;
        std::transform(std::begin(value), std::end(value), std::begin(result),
                       RosConverter<CppItem, RosItem>::to_ros);
        return result;
    }

    static std::array<CppItem, size> from_ros(
        const std::array<RosItem, size>& value) {
        std::array<CppItem, size> result;
        std::transform(std::begin(value), std::end(value), std::begin(result),
                       RosConverter<CppItem, RosItem>::from_ros);
        return result;
    }
};

template <typename CppType, typename RosType>
void to_ros(const CppType& from, RosType* to) {
    *to = RosConverter<CppType, RosType>::to_ros(from);
}

template <typename CppType, typename RosType>
RosType to_ros(const CppType& from) {
    return RosConverter<CppType, RosType>::to_ros(from);
}

template <typename CppType, typename RosType>
void from_ros(const RosType& from, CppType* to) {
    *to = RosConverter<CppType, RosType>::from_ros(from);
}

template <typename CppType, typename RosType>
CppType from_ros(const RosType& from) {
    return RosConverter<CppType, RosType>::from_ros(from);
}
