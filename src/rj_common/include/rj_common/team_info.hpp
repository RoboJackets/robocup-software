#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_geometry/vector_type_adapter.hpp>
#include <rj_msgs/msg/team_info.hpp>
#include <rj_protos/state/ssl_gc_referee_message.pb.h>

// Information about a single team.
class TeamInfo {
public:
    using Msg = rj_msgs::msg::TeamInfo;
    using SharedPtr = std::shared_ptr<TeamInfo>;
    using ConstSharedPtr = std::shared_ptr<const TeamInfo>;

    // The team's name (empty string if operator has not typed anything).
    std::string name;
    // The number of goals scored by the team during normal play and overtime.
    int score = 0;
    // The number of red cards issued to the team since the beginning of the
    // game.
    size_t red_cards = 0;
    // The amount of time (in microseconds) left on each yellow card issued to
    // the team.
    // If no yellow cards are issued, this array has no elements.
    // Otherwise, times are ordered from smallest to largest.
    std::vector<RJ::Seconds> yellow_card_times;
    // The total number of yellow cards ever issued to the team.
    size_t yellow_cards = 0;
    // The number of timeouts this team can still call.
    // If in a timeout right now, that timeout is excluded.
    size_t timeouts_left = 0;
    // The duration of timeout this team can use.
    RJ::Seconds timeout_time{0};
    // The pattern number of this team's goalie.
    uint8_t goalie = 0;

    bool operator==(const TeamInfo& other) const {
        return name == other.name && score == other.score &&
               red_cards == other.red_cards &&
               yellow_card_times == other.yellow_card_times &&
               yellow_cards == other.yellow_cards &&
               timeouts_left == other.timeouts_left &&
               timeout_time == other.timeout_time && goalie == other.goalie;
    }

    bool operator!=(const TeamInfo& other) const { return !(*this == other); }

    static TeamInfo from_refbox_packet(const Referee_TeamInfo& packet) {
        TeamInfo info;
        info.name = packet.name();
        info.score = packet.score();
        info.red_cards = packet.red_cards();
        info.yellow_cards = packet.yellow_cards();
        info.yellow_card_times.resize(info.yellow_cards);
        for (int i = 0; i < packet.yellow_card_times_size(); i++) {
            info.yellow_card_times[i] =
                std::chrono::microseconds(packet.yellow_card_times(i));
        }
        info.timeouts_left = packet.timeouts();
        info.timeout_time = std::chrono::microseconds(packet.timeout_time());
        info.goalie = packet.goalkeeper();
        return info;
    }
};

namespace rclcpp {

template <>
struct TypeAdapter<TeamInfo, rj_msgs::msg::TeamInfo> {
    using is_specialized = std::true_type;
    using custom_type = TeamInfo;
    using ros_message_type = rj_msgs::msg::TeamInfo;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination.name = source.name;
        destination.score = source.score;
        destination.num_red_cards = source.red_cards;
        destination.num_yellow_cards = source.yellow_cards;
        rclcpp::TypeAdapter<std::vector<RJ::Seconds>,
                            std::vector<builtin_interfaces::msg::Duration>>::convert_to_ros_message(
            source.yellow_card_times, destination.yellow_card_remaining_times);
        destination.timeouts_left = source.timeouts_left;
        rclcpp::TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration>::convert_to_ros_message(
            source.timeout_time, destination.remaining_timeout_time);
        destination.goalie_id = source.goalie;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.name = source.name;
        destination.score = source.score;
        destination.red_cards = source.num_red_cards;
        destination.yellow_cards = source.num_yellow_cards;
        rclcpp::TypeAdapter<std::vector<RJ::Seconds>,
                            std::vector<builtin_interfaces::msg::Duration>>::convert_to_custom(
            source.yellow_card_remaining_times, destination.yellow_card_times);
        destination.timeouts_left = source.timeouts_left;
        rclcpp::TypeAdapter<RJ::Seconds, builtin_interfaces::msg::Duration>::convert_to_custom(
            source.remaining_timeout_time, destination.timeout_time);
        destination.goalie = source.goalie_id;
    }
};

}  // namespace rclcpp

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(TeamInfo, rj_msgs::msg::TeamInfo);
