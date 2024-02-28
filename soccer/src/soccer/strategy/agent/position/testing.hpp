#pragma once

#include <algorithm>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>

#include "offense.hpp"
#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class Testing : public Position {
public:
    Testing(int r_id);
    ~Testing() override = default;

private:
    enum State {
        NULLOPT,
        IDLING,
        BASIC_MOVEMENT_1,  // move robot to location
        BASIC_MOVEMENT_2,  // move robot in straight line
        BASIC_MOVEMENT_3,  // move robot back in straight line while staring at a fixed point
        BRING_TO_CENTER,   // bring ball to center
        SPIN_1,            // spin 180
        SPIN_2,            // spin other 180
        LINEKICK,          // test line kick by kicking in goal
    };

    inline static std::vector<bool> is_running_;
    inline static int counter = 0;
    bool move_on_ = false;
    int r_id_;
    inline static bool lock_ = false;
    inline static bool lock2_ = false;

    State current_state_ = IDLING;
    State next_ = NULLOPT;
    Offense off_;

    State update_state();
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    bool proceed();
    bool derived_check();
    bool start();
    bool is_ball_near_center();
};

}  // namespace strategy