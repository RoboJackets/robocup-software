#pragma once

#include <rclcpp/time.hpp>
#include <type_traits>

namespace rj_convert {

template <typename CppType, typename RosType>
struct RosConverter {};

#define CONVERT_PRIMITIVE(type)                                   \
    template <>                                                   \
    struct RosConverter<type, type> {                             \
        static type to_ros(const type& value) { return value; }   \
        static type from_ros(const type& value) { return value; } \
    }

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(int8_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(int16_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(int32_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(int64_t);

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(uint8_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(uint16_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(uint32_t);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(uint64_t);

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(float);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(double);

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(bool);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(std::string);
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
CONVERT_PRIMITIVE(std::u16string);

#undef CONVERT_PRIMITIVE

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
void convert_to_ros(const CppType& from, RosType* to) {
    *to = RosConverter<CppType, RosType>::to_ros(from);
}

template <typename CppType, typename RosType>
RosType convert_to_ros(const CppType& from) {
    return RosConverter<CppType, RosType>::to_ros(from);
}

template <typename CppType, typename RosType>
void convert_from_ros(const RosType& from, CppType* to) {
    *to = RosConverter<CppType, RosType>::from_ros(from);
}

template <typename CppType, typename RosType>
CppType convert_from_ros(const RosType& from) {
    return RosConverter<CppType, RosType>::from_ros(from);
}

}  // namespace rj_convert