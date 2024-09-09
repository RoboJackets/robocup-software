#pragma once

#include "path_planner.hpp"
#include "path_target_path_planner.hpp"

namespace planning {
    /**
     * Path planner that only rotates the robot about a point given.
     * 
     * Params taken from MotionCommand:
     *  target.position - robot will face this point when done
     *  target.pivot_point - robot will pivot around this point 
     */
    class RotatePathPlanner : public PathPlanner {
    public:
        RotatePathPlanner() : PathPlanner("rotate") {}
        ~RotatePathPlanner() override = default;

        RotatePathPlanner(RotatePathPlanner&&) noexcept = default;
        RotatePathPlanner& operator=(RotatePathPlanner&&) noexcept = default;
        RotatePathPlanner(const RotatePathPlanner&) = default;
        RotatePathPlanner& operator=(const RotatePathPlanner&) = default;

        Trajectory plan(const PlanRequest& request) override;

        void reset() override {
            cached_target_angle_ = std::nullopt;
            cached_angle_change_ = std::nullopt;
        }
        [[nodiscard]] bool is_done() const override;

    private:
        Trajectory previous_;
        
        std::optional<double> cached_target_angle_; // equivalent to previously recorded accorded
        std::optional<double> cached_angle_change_;

        std::optional<Trajectory> cached_path_;

        PathTargetPathPlanner path_target_{};

        Trajectory pivot(const PlanRequest& request);

        static constexpr double kIsDoneAngleChangeThresh{1.0};
    };
}