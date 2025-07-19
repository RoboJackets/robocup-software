#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class SoloOffense : public Position {
public:
    SoloOffense(const Position& other);
    SoloOffense(int r_id);
    ~SoloOffense() override = default;
    SoloOffense(const SoloOffense& other) = default;
    SoloOffense(SoloOffense&& other) = default;

    std::string get_current_state() override;

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    enum State { DEFAULT, TO_BALL, KICK };
    State current_state_ = DEFAULT;
    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    RJ::Time last_time_;
    void reset_timeout() { last_time_ = RJ::now(); }
    bool timed_out() const {
        using namespace std::chrono_literals;
        return last_time_ + 4s < RJ::now();
    };

    /**
     * @return a good point to shoot at
     */
    rj_geometry::Point calculate_best_shot() const;
    /**
     * @return distance from shot line (head, tail) to nearest opponent
     */
    double distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const;
    /**
     * @return whether a point is in a red zone
     */
    bool point_in_red(rj_geometry::Point concerned_point) const;

    rj_geometry::Point cached_ball_pos_;
    rj_geometry::Point get_ball_pos() const;

    rj_geometry::Point shot_target_;
    static constexpr double kBackOffset = 3.5 * kRobotRadius;
};

}  // namespace strategy
