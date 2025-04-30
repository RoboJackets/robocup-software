#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {
class Idle : public Position {
public:
    Idle(int r_id);
    ~Idle() = default;
    Idle(const Position& other);
    Idle(int r_id, std::shared_ptr<ClientHandles> clientHandles);
    Idle(const Position& other, std::shared_ptr<ClientHandles> clientHandles);

    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_pass() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_pass_ball() override;
    /**
     * @brief Does nothing; this position is a special case
     */
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    std::shared_ptr<ClientHandles> clientHandles_;
};
}  // namespace strategy
