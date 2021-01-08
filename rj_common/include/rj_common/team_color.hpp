#pragma once

#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/team_color.hpp>

enum class TeamColor {
    kBlue,
    kYellow
};

TeamColor opponent(TeamColor team);

namespace rj_convert {

template <>
struct RosConverter<TeamColor, rj_msgs::msg::TeamColor> {
    static rj_msgs::msg::TeamColor to_ros(const TeamColor& from) {
        return rj_msgs::build<rj_msgs::msg::TeamColor>()
            .is_blue(from == TeamColor::kBlue);
    }

    static TeamColor from_ros(const rj_msgs::msg::TeamColor& from) {
        return from.is_blue ? TeamColor::kBlue : TeamColor::kYellow;
    }
};

ASSOCIATE_CPP_ROS(TeamColor, rj_msgs::msg::TeamColor);

}  // namespace rj_convert

