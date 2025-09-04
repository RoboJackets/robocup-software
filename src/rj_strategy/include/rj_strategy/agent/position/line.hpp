#pragma once

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include "rj_strategy/agent/position.hpp"

namespace strategy {
class Line : public Position {
public:
    Line(const Position& other);
    Line(int r_id);
    Line(int r_id, bool forward);
    ~Line() override = default;
    Line(const Line& other) = default;
    Line(Line&& other) = default;

    std::string get_current_state() override { return "Line"; }

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    bool forward_ = true;
    bool vertical_ = false;
    bool face_target_ = false;
};
}  // namespace strategy
