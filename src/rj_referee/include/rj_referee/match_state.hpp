#pragma once

#include <memory>

#include <rj_referee_msgs/msg/match_state.hpp>
#include <rj_ros_common/time_adapters.hpp>

struct MatchState {
public:
    using Msg = rj_referee_msgs::msg::MatchState;
    using SharedPtr = std::shared_ptr<MatchState>;
    using ConstSharedPtr = std::shared_ptr<const MatchState>;

    enum Period { FirstHalf, Halftime, SecondHalf, Overtime1, Overtime2, PenaltyShootout };

    Period period;

    // Time in seconds remaining in the current period
    RJ::Seconds stage_time_left{};
};

namespace rclcpp {

template <>
struct TypeAdapter<MatchState::Period, uint8_t> {
    using is_specialized = std::true_type;
    using custom_type = MatchState::Period;
    using ros_message_type = uint8_t;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = static_cast<MatchState::Period>(source);
    }
};

template <>
struct TypeAdapter<MatchState, rj_referee_msgs::msg::MatchState> {
    using is_specialized = std::true_type;
    using custom_type = MatchState;
    using ros_message_type = rj_referee_msgs::msg::MatchState;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        rclcpp::TypeAdapter<MatchState::Period, uint8_t>::convert_to_ros_message(
            source.period, destination.period);
        rclcpp::TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration>::convert_to_ros_message(
            source.stage_time_left, destination.stage_time_left);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rclcpp::TypeAdapter<MatchState::Period, uint8_t>::convert_to_custom(source.period,
                                                                            destination.period);
        rclcpp::TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration>::convert_to_custom(
            source.stage_time_left, destination.stage_time_left);
    }
};

}  // namespace rclcpp
