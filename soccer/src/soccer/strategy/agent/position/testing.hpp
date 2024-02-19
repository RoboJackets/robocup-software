#pragma once

#include <spdlog/spdlog.h>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rj_common/time.hpp>

#include "rj_geometry/point.hpp"

namespace strategy {

class Testing : public Position {

public:
    Testing(int r_id);
    ~Testing() override = default;

private:

    enum State {
        IDLING,
        BASIC_MOVEMENT_1, // move robot to location
        BASIC_MOVEMENT_2, // move robot in straight line
    };

    State current_state_ = IDLING;
    RJ::Timestamp time_stamp_;

    State update_state();
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    bool move_on_ = false;

};

} // namespace strategy