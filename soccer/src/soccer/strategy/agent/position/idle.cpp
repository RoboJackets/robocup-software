#include <rclcpp/rclcpp.hpp>

#include "position.hpp"
#include "idle.hpp"

namespace strategy {

    Idle::Idle(int r_id) : Position {r_id} {
        position_name_ = "Idle";
                        SPDLOG_INFO("Robot {} is now Idle", r_id);

    }

    /**
     * @brief Does nothing; this position is a special case
     */
    void Idle::derived_acknowledge_pass() {

    }
    /**
     * @brief Does nothing; this position is a special case
     */
    void Idle::derived_pass_ball() {

    };
    /**
     * @brief Does nothing; this position is a special case
     */
    void Idle::derived_acknowledge_ball_in_transit() {

    }

    std::optional<RobotIntent> Idle::derived_get_task(RobotIntent intent) {
        return intent;
    };

}

