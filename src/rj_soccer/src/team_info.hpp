#pragma once

#include <rj_protos/referee.pb.h>

#include <rj_common/time.hpp>
#include <rj_msgs/msg/team_info.hpp>
#include <string>
#include <vector>

// Information about a single team.
class TeamInfo {
public:
    using Msg = rj_msgs::msg::TeamInfo;

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

namespace rj_convert {

template <>
struct RosConverter<TeamInfo, rj_msgs::msg::TeamInfo> {
    static rj_msgs::msg::TeamInfo to_ros(const TeamInfo& from) {
        rj_msgs::msg::TeamInfo to;
        convert_to_ros(from.name, &to.name);
        convert_to_ros(from.score, &to.score);
        convert_to_ros(from.red_cards, &to.num_red_cards);
        convert_to_ros(from.yellow_cards, &to.num_yellow_cards);
        convert_to_ros(from.yellow_card_times, &to.yellow_card_remaining_times);
        convert_to_ros(from.timeouts_left, &to.timeouts_left);
        convert_to_ros(from.timeout_time, &to.remaining_timeout_time);
        convert_to_ros(from.goalie, &to.goalie_id);
        return to;
    }

    static TeamInfo from_ros(const rj_msgs::msg::TeamInfo& from) {
        TeamInfo to;
        convert_from_ros(from.name, &to.name);
        convert_from_ros(from.score, &to.score);
        convert_from_ros(from.num_red_cards, &to.red_cards);
        convert_from_ros(from.num_yellow_cards, &to.yellow_cards);
        convert_from_ros(from.yellow_card_remaining_times,
                         &to.yellow_card_times);
        convert_from_ros(from.timeouts_left, &to.timeouts_left);
        convert_from_ros(from.remaining_timeout_time, &to.timeout_time);
        convert_from_ros(from.goalie_id, &to.goalie);
        return to;
    }
};

ASSOCIATE_CPP_ROS(TeamInfo, TeamInfo::Msg);

}  // namespace rj_convert