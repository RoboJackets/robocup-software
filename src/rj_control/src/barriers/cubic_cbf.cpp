#include "rj_control/barriers/cubic_cbf.hpp"

namespace control {

CubicCBF::CubicCBF(int robot_id, double barrier_gain, double dt, bool goalie)
    : BarrierCertificate(robot_id, barrier_gain, dt, goalie)
{
    P_ = Eigen::SparseMatrix<double>(2, 2);
    P_.insert(0, 0) = 2;
    P_.insert(1, 1) = 2;
    P_.makeCompressed();
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
}

rj_geometry::Twist CubicCBF::apply(
    const WorldState& world_state,
    const PlayState& play_state,
    const FieldDimensions& field_dimensions,
    const rj_geometry::Twist& control_input,
    bool avoid_ball 
) {
    q_ = -2 * control_input.linear();

    calculate_robot_constraints(world_state, play_state, A_, l_, u_);

    Eigen::SparseMatrix<double> a_sparse = A_.sparseView();

    if (!solver_.isInitialized()) {
        OsqpEigen::Data problem_data;
        solver_.data()->setNumberOfVariables(2);
        solver_.data()->setNumberOfConstraints(kConstraints);
        solver_.data()->setHessianMatrix(P_);
        solver_.data()->setLinearConstraintsMatrix(a_sparse);
        solver_.data()->setGradient(q_);
        solver_.data()->setBounds(l_, u_);

        if (!solver_.initSolver()) {
            SPDLOG_ERROR("Unable to Initialize Barrier Solver");
            return {0, 0, 0};
        }
    } else {
        solver_.updateBounds(l_, u_);
        solver_.updateLinearConstraintsMatrix(a_sparse);
        solver_.updateGradient(q_);
    }

    OsqpEigen::ErrorExitFlag exit_flag = solver_.solveProblem();
    if (exit_flag != OsqpEigen::ErrorExitFlag::NoError) {
        SPDLOG_ERROR("Unable to Solve Barriers");
        return {0, 0, 0};
    }

    auto solution = solver_.getSolution();
    return {solution(0), solution(1), control_input.angular()};
}
    
void CubicCBF::calculate_robot_constraints(
    const WorldState& world_state,
    const PlayState& play_state,
    Eigen::Matrix<double, kConstraints, 2>& A,
    Eigen::Matrix<double, kConstraints, 1>& l,
    Eigen::Matrix<double, kConstraints, 1>& u
) const {
    Eigen::Vector2d our_pose = world_state.get_robot(true, robot_id_).pose.position();

    // Calculate our robot constraints
    for (int j = 0; static_cast<size_t>(j) < kNumShells; j++) {
        if (j == robot_id_) {
            A(j, 0) = 0;
            A(j, 1) = 0;
            l(j) = -1e9;
            u(j) = 1e9;
            continue;
        }

        if (world_state.get_robot(true, j).visible) {
            Eigen::Vector2d their_pose = world_state.get_robot(true, j).pose.position();

            Eigen::Vector2d error = our_pose - their_pose;

            double h = std::pow(error.x(), 2) + std::pow(error.y(), 2) - std::pow(kSafetyRadius, 2);
            double L_g1 = 2 * error.x();
            double L_g2 = 2 * error.y();

            A(j, 0) = -L_g1;
            A(j, 1) = -L_g2;
            l(j) = -1e9;
            u(j) = control_gain_ * std::pow(h, 3);
        } else {
            A(j, 0) = 0;
            A(j, 1) = 0;
            l(j) = -1e9;
            u(j) = 1e9;
        }
    }

    // Calculate their robot constraints
    for (int j = 0; static_cast<size_t>(j) < kNumShells; j++) {
        if (world_state.get_robot(false, j).visible) {
            Eigen::Vector2d their_pose = world_state.get_robot(false, j).pose.position();
            Eigen::Vector2d error = our_pose - their_pose;

            double h = std::pow(error.x(), 2) + std::pow(error.y(), 2) - std::pow(kSafetyRadius, 2);
            double L_g1 = 2 * error.x();
            double L_g2 = 2 * error.y();

            A(kNumShells + j, 0) = -L_g1;
            A(kNumShells + j, 1) = -L_g2;
            l(kNumShells + j) = -1e9;
            u(kNumShells + j) = control_gain_ * std::pow(h, 3);
        } else {
            A(kNumShells + j, 0) = 0;
            A(kNumShells + j, 1) = 0;
            l(kNumShells + j) = -1e9;
            u(kNumShells + j) = 1e9;
        }
    }
}

} // namespace control