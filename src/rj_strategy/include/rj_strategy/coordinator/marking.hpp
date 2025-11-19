#pragma once

#include <algorithm>
#include <array>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/marking.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_msgs/srv/marking.hpp>

#include "rj_strategy/coordinator.hpp"

namespace strategy {

class Marking : public Coordinator<Marking, rj_msgs::srv::Marking, rj_msgs::msg::Marking> {
public:
    static constexpr uint8_t kInvalidRobotId = kNumShells;

    Marking();
    ~Marking() override = default;
    Marking(const Marking&) = delete;
    Marking& operator=(const Marking&) = delete;
    Marking(Marking&&) = delete;
    Marking& operator=(Marking&&) = delete;

    void service_callback(RequestPtr request, ResponsePtr response);

private:
    void publish_marking_list();
    void update_danger_scores();
    uint8_t find_their_robot_in_possession();
    uint8_t most_dangerous_robot(uint8_t robotInPossession);

    static constexpr int kMaxMarkers = 2;
    // this is the threshold value for switching
    // you can play with these constants if the current results aren't good enough
    static constexpr double kSuperDangerSub = 3.2;
    static constexpr double kDangerDistToBall = 3.0;
    static constexpr double kDangerDistToGoal = 5.0;
    static constexpr double kDangerDistToOurRobots = 3.0;
    static constexpr double kDangerAngle = 2.0;
    static constexpr double kPossessionThreshold = 0.3;
    int num_markers_;

    std::array<uint8_t, kNumShells> marking_list_{};
    std::array<double, kNumShells> danger_score_{};
    std::array<uint8_t, kNumShells> enemy_to_friends_{};
    std::vector<uint8_t> unassigned_markers_queue_;
    WorldState last_world_state_;
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
};

}  // namespace strategy
