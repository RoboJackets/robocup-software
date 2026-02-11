#pragma once

#include <rj_geometry/point.hpp>

#include <rj_constants/constants.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/replanner.hpp"

namespace planning {

/**
 * @brief PathPlanner that approaches and captures the ball with a simple state machine.
 *
 * Params taken from MotionCommand:
 *   None
 */
class CollectPathPlanner : public PathPlanner {
public:
    CollectPathPlanner() : PathPlanner("collect") {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    enum class State {
        APPROACH,
        CAPTURE,
    };

    [[nodiscard]] rj_geometry::Point compute_approach_direction(
        const RobotInstant& start, const rj_geometry::Point& ball_position) const;

    void update_state(const PlanRequest& request, const rj_geometry::Point& ball_position);

    Trajectory plan_approach(const PlanRequest& request,
                             const rj_geometry::Point& approach_direction);

    Trajectory plan_capture(const PlanRequest& request,
                            const rj_geometry::Point& approach_direction);

    Trajectory build_trajectory(const PlanRequest& request, const rj_geometry::Point& target_pos,
                                const rj_geometry::Point& target_vel, double accel_scale,
                                const char* debug_label);

    State current_state_{State::APPROACH};

    rj_geometry::Point filtered_ball_velocity_{0, 0};
    bool filtered_ball_velocity_initialized_{false};
    bool is_ball_sense_{false};

    static constexpr double kDirectionEpsilon{1e-3};
    static constexpr double kCaptureExitHysteresis{0.05};
};

}  // namespace planning
