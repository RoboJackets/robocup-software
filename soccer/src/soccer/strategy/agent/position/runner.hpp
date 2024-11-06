#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "marker.hpp"
#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    std::string get_current_state() override;

private:
    // static constexpr int kMaxWallers{6};
    static constexpr int kMaxWallers{
        static_cast<int>(kNumShells)};  // This effectively turns off marking

    /**
     * @brief The derived_get_task method returns the task for the defensive robot
     *  to do based on the game situation. The method will continuously look to assign
     *  the robot to a defensive role, allowing us to quickly swap roles through
     *  gametime. Roles aim to include things such as waller, blocker, etc.
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        RUNNING,  // running along a line
        TURNING   // tuning 90 degrees

    };

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = RUNNING;
};

}  // namespace strategy
