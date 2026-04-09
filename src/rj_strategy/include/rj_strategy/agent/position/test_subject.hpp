#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {

/**
 * Empty scaffold role for quick role-assignment testing in RobotFactoryPosition.
 */
class TestSubject : public Position {
public:
    explicit TestSubject(int r_id);
    ~TestSubject() override = default;
    TestSubject(Position&& other);

    std::string get_current_state() override;

private:
    enum State { IDLE, YELLOW_STRAIGHT, BLUE_STRAIGHT, BORDER_FIELD };

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    State next_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = IDLE;

    // Cached each tick from Position::motion_test_type(), which is set upstream from
    // motion_test_type_from_int(...) in AgentActionClient.
    MotionTestType motion_test_type_value_{MotionTestType::NONE};
};

}  // namespace strategy
