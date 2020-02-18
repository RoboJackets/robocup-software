#pragma once

#include <vector>

#include "Planner.hpp"

namespace Planning {

/**
 * A MPC-based planner using iLQR to calculate optimal feedback trajectories.
 *
 * State: x y h dx dy dh
 *
 * Dynamics:
 *      x' = x + dx*dt  y' = y + dy*dt  h' = h + dh*dt
 *      dx' = u0*dt     dy' = u1*dt     dh' = u2 * dt
 */
class MPCPlanner : public PlannerForCommandType<PathTargetCommand> {
public:
    MPCPlanner() : plan_cache(Num_Shells) {}

    bool isApplicable(const MotionCommand& command) const override;
    Trajectory plan(PlanRequest&& request) override;
    std::string name() const override;

private:
    template<typename Matrix>
    using EigenAlignedVector = std::vector<Matrix, Eigen::aligned_allocator<Matrix>>;

    constexpr static double dt = 0.1;

    struct Instant {
        Eigen::Matrix<double, 6, 1> x_nominal;
        Eigen::Matrix<double, 3, 1> u_nominal;
        Eigen::Matrix<double, 3, 6> K;
    };

    struct PlanCache {
        EigenAlignedVector<Instant> instants;
        RJ::Time last_update;
    };

    /**
     * Calculate the system's coefficients evaluated around a particular value
     * of x.
     */
    void calculateLinearQuadratic(
            Eigen::Matrix<double, 6, 6>* A,
            Eigen::Matrix<double, 6, 3>* B,
            Eigen::Matrix<double, 6, 1>* c,
            Eigen::Matrix<double, 6, 6>* Q,
            Eigen::Matrix<double, 1, 6>* q,
            Eigen::Matrix<double, 3, 3>* R,
            Eigen::Matrix<double, 1, 3>* r,
            Eigen::Matrix<double, 6, 1> x,
            const EigenAlignedVector<Eigen::Matrix<double, 2, 1>>& obstacles
            ) const;

    void calculateQuadraticFinal(
            Eigen::Matrix<double, 6, 6>* Q,
            Eigen::Matrix<double, 1, 6>* q,
            double* cost_final,
            Eigen::Matrix<double, 6, 1> x,
            Eigen::Matrix<double, 6, 1> x_target
            );

    std::vector<PlanCache> plan_cache;
};

}
