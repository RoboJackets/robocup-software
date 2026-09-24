#pragma once

#include <rclcpp/type_adapter.hpp>

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace rclcpp {

namespace detail {

template <typename CustomItem, typename RosItem>
struct vector_item_is_adaptable
    : std::bool_constant<std::is_same_v<CustomItem, RosItem> ||
                         TypeAdapter<CustomItem, RosItem>::is_specialized::value> {};

}  // namespace detail

template <typename CustomItem, typename RosItem>
struct TypeAdapter<
    std::vector<CustomItem>, std::vector<RosItem>,
    std::enable_if_t<detail::vector_item_is_adaptable<CustomItem, RosItem>::value>> {
    using is_specialized = std::true_type;
    using custom_type = std::vector<CustomItem>;
    using ros_message_type = std::vector<RosItem>;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.clear();
        destination.reserve(source.size());
        for (const auto& item : source) {
            RosItem converted;
            if constexpr (std::is_same_v<CustomItem, RosItem>) {
                converted = item;
            } else {
                TypeAdapter<CustomItem, RosItem>::convert_to_ros(item, converted);
            }
            destination.emplace_back(std::move(converted));
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.clear();
        destination.reserve(source.size());
        for (const auto& item : source) {
            CustomItem converted;
            if constexpr (std::is_same_v<CustomItem, RosItem>) {
                converted = item;
            } else {
                TypeAdapter<CustomItem, RosItem>::convert_to_custom(item, converted);
            }
            destination.emplace_back(std::move(converted));
        }
    }
};

}  // namespace rclcpp

namespace rj_convert {

template <typename CustomType, typename RosType>
RosType convert_to_ros(const CustomType& source) {
    RosType destination;
    if constexpr (std::is_same_v<CustomType, RosType>) {
        destination = source;
    } else {
        rclcpp::TypeAdapter<CustomType, RosType>::convert_to_ros(source, destination);
    }
    return destination;
}

template <typename RosType, typename CustomType>
CustomType convert_from_ros(const RosType& source) {
    CustomType destination;
    if constexpr (std::is_same_v<CustomType, RosType>) {
        destination = source;
    } else {
        rclcpp::TypeAdapter<CustomType, RosType>::convert_to_custom(source, destination);
    }
    return destination;
}

template <typename CustomType, typename RosType>
void convert_to_ros(const CustomType& source, RosType* destination) {
    if constexpr (std::is_same_v<CustomType, RosType>) {
        *destination = source;
    } else {
        rclcpp::TypeAdapter<CustomType, RosType>::convert_to_ros(source, *destination);
    }
}

template <typename RosType, typename CustomType>
void convert_from_ros(const RosType& source, CustomType* destination) {
    if constexpr (std::is_same_v<CustomType, RosType>) {
        *destination = source;
    } else {
        rclcpp::TypeAdapter<CustomType, RosType>::convert_to_custom(source, *destination);
    }
}

}  // namespace rj_convert
