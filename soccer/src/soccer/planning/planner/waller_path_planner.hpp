#pragma once

#include "path_planner.hpp"
#include "path_target_path_planner.hpp"

namespace planning {

/**
 * PathPlanner which governs the movement of the wallers
 *
 * Each waller will follow the waller in front of it,
 * ensuring that they move in a smooth path
 *
 * Params taken from MotionCommand:
 *   command.waller_radius - radius of the waller arc
 *   command.walling_robots - vector of current wallers
 */    

class WallerPathPlanner : public PathPlanner {
public:
    WallerPathPlanner() : PathPlanner("waller") {}
    ~WallerPathPlanner() override = default;

    WallerPathPlanner(WallerPathPlanner&&) noexcept = default;
    WallerPathPlanner& operator=(WallerPathPlanner&&) noexcept = default;
    WallerPathPlanner(const WallerPathPlanner&) = default;
    WallerPathPlanner& operator=(const WallerPathPlanner&) = default;

    Trajectory plan(const PlanRequest& request) override;

    void reset() override {}

    [[nodiscard]] bool is_done() const override;

private:
    static constexpr double kRobotDiameterMultiplier = 1.5;

    PathTargetPathPlanner path_target_{};
};
} // namespace planning