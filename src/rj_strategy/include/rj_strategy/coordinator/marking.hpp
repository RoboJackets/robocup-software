#pragma once

#include <array>
#include <algorithm>  // for std::any_of
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/world_state.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/marking.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_msgs/srv/marking.hpp>

#include "rj_strategy/coordinator.hpp"


namespace strategy {

class Marking
    : public Coordinator<Marking, rj_msgs::srv::Marking, rj_msgs::msg::Marking> {
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

    static constexpr int kMaxMarkers = 20;
    // this is the threshold value for switching
    static constexpr double kSuperDangerSub = 3.1415926535;
    static constexpr double kDangerDistToBall = 1.0;
    static constexpr double kDangerDistToGoal = 2.0;
    static constexpr double kDangerDistToOurRobots = 3.0;
    static constexpr double kDangerAngle = 2.0;
    int num_markers_;

    std::array<uint8_t, kNumShells> marking_list_{};  // Initialize it to invalid robot id in constructor
    std::array<double, kNumShells> danger_score_{}; // infinity initialized in constructor, no one is a valid target initially
    std::array<uint8_t, kNumShells> enemey_to_friends_{};
    std::vector<uint8_t> queue_;
    WorldState last_world_state_;
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
};

}  // namespace strategy
