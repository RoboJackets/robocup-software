//
// Created by kyle on 1/13/20.
//

#include "MPCPlanner.h"
#include <limits>

bool Planning::MPCPlanner::isApplicable(
        const Planning::MotionCommand &command) const {
    return std::holds_alternative<PosVelCommand>(command) ||
           std::holds_alternative<PathTargetCommand>(command);
}

Planning::Trajectory
Planning::MPCPlanner::plan(Planning::PlanRequest &&request) {
    int iterations = 0;
    int max_iterations = 5;

    Eigen::Matrix<double, 6, 1> x_target;
    if (std::holds_alternative<PosVelCommand>(request.motionCommand)) {
        PosVelCommand command = std::get<PosVelCommand>(request.motionCommand);
        x_target.head<3>() = (Eigen::Vector3d) command.target;
        x_target.tail<3>() = (Eigen::Vector3d) command.velocity;
    } else if (std::holds_alternative<PathTargetCommand>(request.motionCommand)){
        PathTargetCommand command = std::get<PathTargetCommand>(request.motionCommand);
        x_target.head<3>() = (Eigen::Vector3d) command.pathGoal.pose;
        x_target.tail<3>() = (Eigen::Vector3d) command.pathGoal.velocity;
    }

    bool converged = false;

    int horizon = 20;

    PlanCache& cache = plan_cache[request.shellID];

    // Initialize the cache with zeros if necessary.
    if (cache.instants.empty()) {
        cache.instants.resize(horizon + 1);
        for (int i = 0; i < horizon + 1; i++) {
            cache.instants[i].x_nominal = Eigen::Matrix<double, 6, 1>::Zero();
            cache.instants[i].u_nominal = Eigen::Matrix<double, 3, 1>::Zero();
            cache.instants[i].K = Eigen::Matrix<double, 3, 6>::Zero();
        }
    } else if (cache.last_update + std::chrono::milliseconds(100) > RJ::now()) {
        // If we've calculated a new trajectory recently then we can reuse it
        // TODO: Remove duplicate with the below
        std::list<RobotInstant> instants;
        RJ::Time time = cache.last_update;
        for (int t = 0; t < horizon + 1; t++) {
            RobotInstant instant {
                    Geometry2d::Pose(cache.instants[t].x_nominal.head<3>()),
                    Geometry2d::Twist(cache.instants[t].x_nominal.tail<3>()),
                    time
            };
            instants.push_back(instant);
            time += std::chrono::duration_cast<std::chrono::microseconds>(
                    RJ::Seconds(dt));
        }

        std::cout << "Reusing old trajectory: " << (cache.last_update.time_since_epoch() + std::chrono::milliseconds(1000) > RJ::now().time_since_epoch()) << std::endl;

        Planning::Trajectory trajectory{std::move(instants)};
        return trajectory;
    }

    cache.last_update = RJ::now();

    Eigen::Matrix<double, 6, 1> x_initial;
    x_initial.head<3>() = (Eigen::Vector3d) request.start.pose;
    x_initial.tail<3>() = (Eigen::Vector3d) request.start.velocity;

    EigenAlignedVector <Eigen::Matrix<double, 6, 6>> A(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 6, 3>> B(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 6, 1>> c(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 6, 6>> Q(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 1, 6>> q(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 3, 3>> R(horizon + 1);
    EigenAlignedVector <Eigen::Matrix<double, 1, 3>> r(horizon + 1);

    std::vector<EigenAlignedVector<Eigen::Matrix<double, 2, 1>>> obstacles(horizon + 1);

    double V_prev = std::numeric_limits<double>::infinity();

    while (!converged && iterations < max_iterations) {
        // Forward pass
        Eigen::Matrix<double, 6, 1> x = x_initial;
        cache.instants[0].x_nominal = x;
        for (int t = 0; t < horizon; t++) {
            // Calculate u
            auto u = cache.instants[t].u_nominal + cache.instants[t].K * (x - cache.instants[t].x_nominal) * 0;
            calculateLinearQuadratic(
                    &A[t],
                    &B[t],
                    &c[t],
                    &Q[t],
                    &q[t],
                    &R[t],
                    &r[t],
                    x,
                    obstacles[t]
                    );
            cache.instants[t].u_nominal = u;

            // Dynamics
            x.head<3>() += dt * x.tail<3>();
            x.tail<3>() += dt * (u - 3 * x.tail<3>()) + 0.5 * dt * dt * u;

            cache.instants[t + 1].x_nominal = x;
        }

        double cost_final = 0;

        // Final cost
        calculateQuadraticFinal(
                &Q[horizon],
                &q[horizon],
                &cost_final,
                x,
                x_target);

        Eigen::Matrix<double, 6, 6> Vxx = Q[horizon];
        Eigen::Matrix<double, 1, 6> Vx = q[horizon];
        double V = cost_final;

#if 0
        std::cout << "--------------------------------" << std::endl;
        std::cout << "T=FINAL" << std::endl;
        std::cout << "Vxx: \n" << Vxx << std::endl;
        std::cout << "Vx: \n" << Vx << std::endl;
        std::cout << "V: " << V << std::endl;
#endif

        // Backward pass
        for (int t = horizon - 1; t >= 0; t--) {
            Eigen::Matrix<double, 1, 6> Qx = q[t] + Vx * A[t];
            Eigen::Matrix<double, 1, 3> Qu = r[t] + Vx * B[t];
            Eigen::Matrix<double, 6, 6> Qxx = Q[t] + A[t].transpose() * Vxx * A[t];
            Eigen::Matrix<double, 3, 6> Qux = B[t].transpose() * Vxx * A[t];
            Eigen::Matrix<double, 3, 3> Quu = R[t] + B[t].transpose() * Vxx * B[t];

            Eigen::Matrix<double, 3, 3> Quu_inv = Quu.inverse();
            cache.instants[t].u_nominal += -Quu_inv * Qu.transpose();
            cache.instants[t].K = -Quu_inv * Qux;

            auto K = cache.instants[t].K;
            auto k = cache.instants[t].u_nominal;

            Vx = Qx - Qu * Quu_inv * Qux;
            Vxx = Qxx - Qux.transpose() * Quu_inv * Qux;

#if 0
            std::cout << "--------------------------------" << std::endl;
            std::cout << "t = " << t << std::endl;
            std::cout << "Qx: \n" << Qx << std::endl;
            std::cout << "Qu: \n" << Qu << std::endl;
            std::cout << "Qxx: \n" << Qxx << std::endl;
            std::cout << "Quu: \n" << Quu << std::endl;
            std::cout << "Qux: \n" << Qux << std::endl;
            std::cout << "Vxx: \n" << Vxx << std::endl;
            std::cout << "Vx: \n" << Vx << std::endl;
            std::cout << "V: " << V << std::endl;
#endif
        }

        // Check acceptance
        if (V_prev - V < 1e-3) {
            converged = false;
        }

        V_prev = V;
        iterations++;
    }

    // Convert into a trajectory
    RJ::Time time = RJ::now();

    std::list<RobotInstant> instants;
    for (int t = 0; t < horizon + 1; t++) {
        RobotInstant instant {
            Geometry2d::Pose(cache.instants[t].x_nominal.head<3>()),
            Geometry2d::Twist(cache.instants[t].x_nominal.tail<3>()),
            time
        };
        instants.push_back(instant);
        time += std::chrono::duration_cast<std::chrono::microseconds>(
                RJ::Seconds(dt));
    }

    std::cout << "u = " << cache.instants[0].u_nominal.transpose() << std::endl;

    Planning::Trajectory trajectory{std::move(instants)};
    return trajectory;
}

std::string Planning::MPCPlanner::name() const {
    return "MPCPlanner";
}

void
Planning::MPCPlanner::calculateLinearQuadratic(Eigen::Matrix<double, 6, 6>* A,
                                               Eigen::Matrix<double, 6, 3>* B,
                                               Eigen::Matrix<double, 6, 1>* c,
                                               Eigen::Matrix<double, 6, 6>* Q,
                                               Eigen::Matrix<double, 1, 6>* q,
                                               Eigen::Matrix<double, 3, 3>* R,
                                               Eigen::Matrix<double, 1, 3>* r,
                                               Eigen::Matrix<double, 6, 1> x,
                                               const EigenAlignedVector<Eigen::Matrix<double, 2, 1>>& obstacles
                                               ) const {
    // clang-format off
    // Because we choose world-space variables, A and B are easy
    *A << 0, 0, 0, dt, 0, 0,
          0, 0, 0, 0, dt, 0,
          0, 0, 0, 0, 0, dt,
          0, 0, 0, -3 * dt, 0, 0,
          0, 0, 0, 0, -3 * dt, 0,
          0, 0, 0, 0, 0, -3 * dt;
    *A += Eigen::Matrix<double, 6, 6>::Identity();
    *B << 0.5 * dt * dt, 0, 0,
          0, 0.5 * dt * dt, 0,
          0, 0, 0.5 * dt * dt,
          dt, 0, 0,
          0, dt, 0,
          0, 0, dt;
    *c << 0, 0, 0, 0, 0, 0;

    // We also chose our cost function in u to be simple (quadratic)
    *R = 10.0 * dt * Eigen::Matrix<double, 3, 3>::Identity();
    *r = Eigen::Matrix<double, 1, 3>::Zero();

    /*
     * Q is more complicated. The cost function is:
     *  sum over obstacles [[ -k*ln(distance from closest point on obstacle) ]]
     * However, this is non-convex (the Hessian will have negative eigenvalue
     * perpendicular to the radial direction). We can hack together a fix to
     * this by letting Q only exist along the radial direction, and setting its
     * value to zero in the perpendicular direction. This gives a modified cost
     * function of:
     *
     *  C = -k \sum { ln(dx*dx_nom + dy*dy_nom) }
     *
     *  where dx_nom, dy_nom are nominal values.
     *
     * So, the gradient is:
     *  C_x = -k \sum { dx_nom / (dx^2 + dy^2) }
     *  C_y = -k \sum { dy_nom / (dx^2 + dy^2) }
     *
     * And the Hessian is:
     *  C_xx = k \sum { dx_nom^2 / (dx*dx_nom + dy*dy_nom)^2 }
     *  C_xx = k \sum { dx_nom*dy_nom / (dx*dx_nom + dy*dy_nom)^2 }
     *  C_yy = k \sum { dy_nom^2 / (dx*dx_nom + dy*dy_nom)^2 }
     */
    double k = 1 * dt;
    *q = Eigen::Matrix<double, 1, 6>::Zero();
    *Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q->bottomRightCorner<3, 3>() = Eigen::Matrix<double, 3, 3>::Identity() * dt;

    /*
    for (const auto& obs : obstacles) {
        double dx = x(0) - obs(0);
        double dy = x(1) - obs(1);
        double norm_sq = dx * dx + dy * dy;

        (*q)(0) += -k * dx / norm_sq;
        (*q)(1) += -k * dy / norm_sq;
        (*Q)(0, 0) += k * dx * dx / norm_sq * norm_sq;
        (*Q)(0, 1) += k * dx * dy / norm_sq * norm_sq;
        (*Q)(1, 0) += k * dx * dy / norm_sq * norm_sq;
        (*Q)(1, 1) += k * dy * dy / norm_sq * norm_sq;
    }
     */

    // clang-format on
}

void
Planning::MPCPlanner::calculateQuadraticFinal(Eigen::Matrix<double, 6, 6> *Q,
                                              Eigen::Matrix<double, 1, 6> *q,
                                              double *cost_final,
                                              Eigen::Matrix<double, 6, 1> x,
                                              Eigen::Matrix<double, 6, 1> x_target) {
    *Q = (Eigen::Matrix<double, 6, 1>() << 10, 10, 10, 5, 5, 5).finished().asDiagonal();
    *q = *Q * (x - x_target);
    *cost_final = 0.5 * (x - x_target).transpose() * (*Q) * (x - x_target);
}
