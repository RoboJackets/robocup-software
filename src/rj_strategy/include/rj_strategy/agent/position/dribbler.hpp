#pragma once

#include "rj_strategy/agent/position.hpp"
#include "rj_geometry/point.hpp"
#include "rj_geometry/pose.hpp"
#include "rj_constants/constants.hpp"
#include <optional>

namespace strategy {

/**
 * The Dribbler position controls the robot to dribble the ball up the field
 * toward a configurable target point, maintaining possession with smooth motion.
 * Uses a lookahead and ball-correction vector to ensure straight-line dribbling
 * and maintain possession at adjustable speed from any starting position.
 */
class Dribbler : public Position {
public:
    explicit Dribbler(int r_id);

    std::string get_current_state() override;

    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    /**
     * @brief Set the target point for dribbling
     * @param target Target position to dribble toward
     */
    void set_dribble_target(rj_geometry::Point target);

    /**
     * @brief Set the dribbler motor speed (0.0 to 1.0)
     * @param speed Motor speed multiplier (0.0 = off, 1.0 = max)
     */
    void set_dribbler_speed(float speed);

protected:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

private:
    bool has_ball() const;
    double distance_to_ball() const;
    bool can_collect_ball() const;

    // Maximum distance to consider the robot as having secure possession
    static constexpr double kOwnBallRadius{kRobotRadius + 0.1};
    
    // Distance at which robot should start collecting ball (must be > kOwnBallRadius)
    static constexpr double kCollectionDistance{kRobotRadius + 0.2};
    
    // Hysteresis: once possession is lost, require this distance for re-collection attempt
    static constexpr double kReacquisitionDistance{kRobotRadius + 0.15};
    
    // Dribble target position (default: goal at far end of field)
    rj_geometry::Point target_point_{0, 9};
    
    // Dribbler motor speed multiplier (0.0 to 1.0)
    float dribbler_speed_{0.8f};
    
    // Track if we had ball in previous cycle (for hysteresis)
    mutable bool had_ball_last_cycle_{false};
};

}  // namespace strategy