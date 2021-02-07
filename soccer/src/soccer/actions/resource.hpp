#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/resource.hpp>

namespace actions {

enum class Subsystem {
    kDrive,
    kDribbler,
    kKicker,
};

struct Resource {
    size_t robot_id;
    Subsystem subsystem;
};

bool operator==(const Resource& a, const Resource& b) {
    return a.robot_id == b.robot_id && a.subsystem == b.subsystem;
}

struct ResourceHash {
    size_t operator()(const actions::Resource& res) const {
        return (std::hash<size_t>{}(res.robot_id) << 1) ^
               (std::hash<actions::Subsystem>{}(res.subsystem));
    }
};

}  // namespace actions

namespace rj_convert {

template <>
struct RosConverter<actions::Resource, rj_msgs::msg::Resource> {
    static rj_msgs::msg::Resource to_ros(const actions::Resource& from) {
        return rj_msgs::build<rj_msgs::msg::Resource>()
            .robot_id(from.robot_id)
            .subsystem(static_cast<int8_t>(from.subsystem));
    }

    static actions::Resource from_ros(const rj_msgs::msg::Resource& from) {
        actions::Subsystem subsystem = actions::Subsystem::kDrive;
        switch (from.subsystem) {
            case rj_msgs::msg::Resource::SUBSYSTEM_DRIVE:
                subsystem = actions::Subsystem::kDrive;
                break;
            case rj_msgs::msg::Resource::SUBSYSTEM_DRIBBLER:
                subsystem = actions::Subsystem::kDribbler;
                break;
            case rj_msgs::msg::Resource::SUBSYSTEM_KICKER:
                subsystem = actions::Subsystem::kKicker;
                break;
        }
        return actions::Resource{from.robot_id, subsystem};
    }
};

ASSOCIATE_CPP_ROS(actions::Resource, rj_msgs::msg::Resource);

}  // namespace rj_convert
