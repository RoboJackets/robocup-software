#pragma once

#include <rclcpp/rclcpp.hpp>

namespace strategy {

class PenaltyPlayer : public Position {
public:
    PenaltyPlayer(int r_id);
    ~PenaltyPlayer() = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy
