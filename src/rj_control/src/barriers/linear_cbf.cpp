#include "rj_control/barriers/linear_cbf.hpp"

namespace control {

//NOLINTNEXTLINE(bugprone-easily-swappable-parameters, readability-identifier-length)
LinearCBF::LinearCBF(int robot_id, double control_gain, double dt, bool goalie)
    : BarrierCertificate(robot_id, control_gain, dt, goalie) {
    P_ = Eigen::SparseMatrix<double>(2, 2);
    P_.insert(0, 0) = 2;
    P_.insert(1, 1) = 2;
    P_.makeCompressed();
    A_ = Eigen::SparseMatrix<double>(kConstraints, 2);
    q_ << 0, 0;
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setVerbosity(false);
}

rj_geometry::Twist LinearCBF::apply(
    const WorldState& world_state,
    const PlayState& play_state,
    const FieldDimensions& field_dimensions,
    const rj_geometry::Twist& control_input,
    bool avoid_ball
) {
    q_ = -2 * control_input.linear();

    // Calculate the constraints placed by the robot positions
    calculate_robot_constraints(world_state, play_state, A_, l_, u_);

    // Calculate the constraints placed by our goal
    calculate_our_goal_constraints(world_state, play_state, field_dimensions, A_, l_, u_);

    // Calculate the constraints placed by their goal
    calculate_their_goal_constraints(world_state, play_state, field_dimensions, A_, l_, u_);

    // Calculate the constraints placed by the ball's position
    calculate_ball_position_constraints(world_state, play_state, A_, l_, u_, avoid_ball);

    // Calculate the constraints placed by ball placement
    calculate_ball_placement_constraints(world_state, play_state, A_, l_, u_);

    // Calculate the velocity constraints placed by the current play state
    calculate_velocity_constraints(play_state, A_, l_, u_);

    Eigen::SparseMatrix<double> a_sparse = A_.sparseView();

    // Initialize or update the solver
    if (!solver_.isInitialized()) {
        OsqpEigen::Data problem_data;
        solver_.data()->setNumberOfVariables(2);
        solver_.data()->setNumberOfConstraints(kConstraints);
        solver_.data()->setHessianMatrix(P_);
        solver_.data()->setLinearConstraintsMatrix(a_sparse);
        solver_.data()->setGradient(q_);
        solver_.data()->setBounds(l_, u_);

        if (!solver_.initSolver()) {
            // TODO (Nathaniel Wert): Handle Solver Initialization Error
            SPDLOG_ERROR("Unable to Initialize Barrier Solver");
            return {0, 0, 0};
        }
    } else {
        solver_.updateBounds(l_, u_);
        solver_.updateLinearConstraintsMatrix(a_sparse);
        solver_.updateGradient(q_);
    }

    // Solve the QP
    OsqpEigen::ErrorExitFlag exit_flag = solver_.solveProblem();
    if (exit_flag != OsqpEigen::ErrorExitFlag::NoError) {
        // TODO (Nathaniel Wert): Handle solver error (in this case their is no control input that keeps us in the safe set)
        SPDLOG_ERROR("Unable to Solve Barriers");
        return { 0, 0, 0};
    }

    auto solution = solver_.getSolution();
    return { solution(0), solution(1), control_input.angular() };
}

void LinearCBF::calculate_robot_constraints(
    const WorldState& world_state,
    const PlayState& play_state,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
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
            Eigen::Vector2d their_velocity = world_state.get_robot(true, j).velocity.linear();

            Eigen::Vector2d delta_p_ij = our_pose - their_pose;

            A(j, 0) = -delta_p_ij(0);
            A(j, 1) = -delta_p_ij(1);
            
            // Calculate the robot barrier
            if (delta_p_ij.norm() < kSafeDistanceRobots + play_state.max_speed() * dt_) {
                double h_ij = delta_p_ij.norm() - kSafeDistanceRobots;
                u(j) = (-delta_p_ij.transpose() * their_velocity)(0) + delta_p_ij.norm() * control_gain_ * h_ij;
            } else {
                u(j) = 1e9;
            }

            l(j) = -1e9;
        } else {
            A(j, 0) = -1.0;
            A(j, 1) = -1.0;
            l(j) = -1e9;
            u(j) = 1e9;
        }
    }

    // Calculate their robot constraints
    for (int j = 0; static_cast<size_t>(j) < kNumShells; j++) {
        if (world_state.get_robot(false, j).visible) {
            Eigen::Vector2d their_pose = world_state.get_robot(false, j).pose.position();
            Eigen::Vector2d their_velocity = world_state.get_robot(false, j).velocity.linear();

            Eigen::Vector2d delta_p_ij = our_pose - their_pose;

            A(static_cast<int>(kTheirRobotsConstraintIdx) + j, 0) = -delta_p_ij(0);
            A(static_cast<int>(kTheirRobotsConstraintIdx) + j, 1) = -delta_p_ij(1);

            // Calculate the robot barrier
            if (delta_p_ij.norm() < kSafeDistanceRobots + play_state.max_speed() * dt_) {
                double h_ij = delta_p_ij.norm() - kSafeDistanceRobots;
                u(static_cast<int>(kTheirRobotsConstraintIdx) + j) = (-delta_p_ij.transpose() * their_velocity)(0) + delta_p_ij.norm() * control_gain_ * h_ij;
            } else {
                u(static_cast<int>(kTheirRobotsConstraintIdx) + j) = 1e9;
            }

            l(static_cast<int>(kTheirRobotsConstraintIdx) + j) = -1e9;
        } else {
            A(static_cast<int>(kTheirRobotsConstraintIdx) + j, 0) = -1.0;
            A(static_cast<int>(kTheirRobotsConstraintIdx) + j, 1) = -1.0;
            l(static_cast<int>(kTheirRobotsConstraintIdx) + j) = -1e9;
            u(static_cast<int>(kTheirRobotsConstraintIdx) + j) = 1e9;
        }
    }
}

void LinearCBF::calculate_our_goal_constraints(
    const WorldState& world_state,
    const PlayState& play_state,
    const FieldDimensions& field_dimensions,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
) const {
    Eigen::Vector2d robot_position = world_state.get_robot(true, robot_id_).pose.position();
    double safe_distance = kSafeDistanceGoal;
    if (play_state.is_stop()) {
        safe_distance += 0.2;
    }
    double apply_distance = safe_distance + play_state.max_speed() * dt_;

    // Are we the top goal
    bool top_goal = field_dimensions.our_goal_loc().y() > field_dimensions.center_field_loc().y();

    // Calculate our goal constraints
    rj_geometry::Rect our_defense_area = field_dimensions.our_defense_area();

    // Are we within the y-values necessary to apply the left and right barriers
    bool within_y = top_goal && robot_position.y() >= our_defense_area.miny() - apply_distance;
    within_y |= !top_goal && robot_position.y() <= our_defense_area.maxy() + apply_distance;

    // Right Goal Area
    A(kOurGoalConstraintsIdx, 0) = -1;
    A(kOurGoalConstraintsIdx, 1) = 0;
    bool apply_right_barrier = 0.0 < robot_position.x()
        && robot_position.x() <= our_defense_area.maxx() + apply_distance;
    double right_distance = robot_position.x() - our_defense_area.maxx();
    if (!goalie_ && within_y && apply_right_barrier) {
        u(kOurGoalConstraintsIdx) = right_distance - safe_distance;
    } else {
        u(kOurGoalConstraintsIdx) = 1e9;
    }
    l(kOurGoalConstraintsIdx) = -1e9;

    // Left Goal Area
    A(kOurGoalConstraintsIdx + 1, 0) = 1;
    A(kOurGoalConstraintsIdx + 1, 1) = 0;
    bool apply_left_barrier = our_defense_area.minx() - apply_distance <= robot_position.x()
        && robot_position.x() < 0.0;
    double left_distance = robot_position.x() - our_defense_area.minx();
    if (!goalie_ && within_y && apply_left_barrier) {
        u(kOurGoalConstraintsIdx + 1) = -(left_distance + safe_distance);
    } else {
        u(kOurGoalConstraintsIdx + 1) = 1e9;
    }
    l(kOurGoalConstraintsIdx + 1) = -1e9;

    // Bottom Goal Area
    A(kOurGoalConstraintsIdx + 2, 0) = 0;
    A(kOurGoalConstraintsIdx + 2, 1) = top_goal ? -1 : 1;

    // Check that we are within the y-values the bottom barrier applies at
    bool apply_bottom_barrier = false;
    if (top_goal) {
        apply_bottom_barrier = our_defense_area.miny() - apply_distance <= robot_position.y()
            && robot_position.y() <= our_defense_area.maxy();
    } else {
        apply_bottom_barrier = our_defense_area.miny() <= robot_position.y()
            && robot_position.y() <= our_defense_area.maxy() + apply_distance;
    }

    // Check that we are in front of the goal area
    bool in_front = our_defense_area.minx() <= robot_position.x()
        && robot_position.x() <= our_defense_area.maxx();
    double bottom_distance = top_goal ? robot_position.y() - our_defense_area.miny() :
        robot_position.y() - our_defense_area.maxy();
    if (
        !goalie_ &&
        in_front &&
        apply_bottom_barrier
    ) {
        u(kOurGoalConstraintsIdx + 2) = top_goal ? -(bottom_distance + safe_distance)
            : bottom_distance - safe_distance;
    } else {
        u(kOurGoalConstraintsIdx + 2) = 1e9;
    }
    l(kOurGoalConstraintsIdx + 2) = -1e9;
}

void LinearCBF::calculate_their_goal_constraints(
    const WorldState& world_state,
    const PlayState& play_state,
    const FieldDimensions& field_dimensions,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
) const {
    Eigen::Vector2d robot_position = world_state.get_robot(true, robot_id_).pose.position();
    double safe_distance = kSafeDistanceGoal;
    if (play_state.is_stop()) {
        safe_distance += 0.2;
    }
    double apply_distance = safe_distance + play_state.max_speed() * dt_;

    // Are we the top goal
    bool top_goal = field_dimensions.their_goal_loc().y() > field_dimensions.center_field_loc().y();
    // Calaculate their goal constraints
    rj_geometry::Rect their_defense_area = field_dimensions.their_defense_area();

    bool within_y = top_goal && robot_position.y() >= their_defense_area.miny() - apply_distance;
    within_y |= !top_goal && robot_position.y() <= their_defense_area.maxy() + apply_distance;

    // Right Goal Area
    A(kTheirGoalConstraintsIdx, 0) = -1;
    A(kTheirGoalConstraintsIdx, 1) = 0;
    bool apply_right_barrier = 0.0 < robot_position.x()
        && robot_position.x() <= their_defense_area.maxx() + apply_distance;
    double right_distance = robot_position.x() - their_defense_area.maxx();
    if (within_y && apply_right_barrier) {
        u(kTheirGoalConstraintsIdx) = right_distance - safe_distance;
    } else {
        u(kTheirGoalConstraintsIdx) = 1e9;
    }
    l(kTheirGoalConstraintsIdx) = -1e9;

    // Left Goal Area
    A(kTheirGoalConstraintsIdx + 1, 0) = 1;
    A(kTheirGoalConstraintsIdx + 1, 1) = 0;
    bool apply_left_barrier = their_defense_area.minx() - apply_distance <= robot_position.x()
        && robot_position.x() < 0.0;
    double left_distance = robot_position.x() - their_defense_area.minx();
    if (within_y && apply_left_barrier) {
        u(kTheirGoalConstraintsIdx + 1) = -(left_distance + safe_distance);
    } else {
        u(kTheirGoalConstraintsIdx + 1) = 1e9;
    }
    l(kTheirGoalConstraintsIdx + 1) = -1e9;

    // Bottom Goal Area
    A(kTheirGoalConstraintsIdx + 2, 0) = 0;
    A(kTheirGoalConstraintsIdx + 2, 1) = top_goal ? -1 : 1;

    // Check that we are within the y-values the bottom barrier applies at
    bool apply_bottom_barrier = false;
    if (top_goal) {
        apply_bottom_barrier = their_defense_area.miny() - apply_distance <= robot_position.y()
            && robot_position.y() <= their_defense_area.maxy();
    } else {
        apply_bottom_barrier = their_defense_area.miny() <= robot_position.y()
            && robot_position.y() <= their_defense_area.maxy() + apply_distance;
    }

    // Check that we are in front of the goal area
    bool in_front = their_defense_area.minx() <= robot_position.x()
        && robot_position.x() <= their_defense_area.maxx();
    double bottom_distance = top_goal ? robot_position.y() - their_defense_area.miny() :
        robot_position.y() - their_defense_area.maxy();
    if (
        in_front &&
        apply_bottom_barrier
    ) {
        u(kTheirGoalConstraintsIdx + 2) = top_goal ? -(bottom_distance + safe_distance)
            : bottom_distance - safe_distance;
    } else {
        u(kTheirGoalConstraintsIdx + 2) = 1e9;
    }
    l(kTheirGoalConstraintsIdx + 2) = -1e9;
}

void LinearCBF::calculate_ball_position_constraints(
    const WorldState& world_state,
    [[maybe_unused]] const PlayState& play_state,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u, //NOLINT(readability-identifier-naming, readability-identifier-length)
    bool avoid_ball
) const {
    Eigen::Vector2d delta_pb = world_state.get_robot(true, robot_id_).pose.position() - world_state.ball.position;

    A(kBallConstraintIdx, 0) = -delta_pb(0);
    A(kBallConstraintIdx, 1) = -delta_pb(1);

    if ((play_state.is_stop() || play_state.is_kickoff() || play_state.is_free_kick() || play_state.is_placement()) && delta_pb.norm() < kSafeDistanceBall + play_state.max_speed() * dt_) {
        double h_b = delta_pb.norm() - kSafeDistanceBall;
        Eigen::Vector2d ball_velocity = world_state.ball.velocity;
        u(kBallConstraintIdx) = (-delta_pb.transpose() * ball_velocity)(0) + delta_pb.norm() * control_gain_ * h_b;
    } else if (avoid_ball) {
        double h_b = delta_pb.norm() - kAvoidBallDistance;
        Eigen::Vector2d ball_velocity = world_state.ball.velocity;
        u(kBallConstraintIdx) = (-delta_pb.transpose() * ball_velocity)(0) + delta_pb.norm() * control_gain_ * h_b;
    } else {
        u(kBallConstraintIdx) = 1e9;
    }

    l(kBallConstraintIdx) = -1e9;
}

void LinearCBF::calculate_ball_placement_constraints(
    const WorldState& world_state,
    [[maybe_unused]] const PlayState& play_state,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
) const {
    Eigen::Vector2d robot_position = world_state.get_robot(true, robot_id_).pose.position();
    if (play_state.is_placement() && !play_state.is_our_restart()) {
        // Calculate Goal Barriers
        Eigen::Vector2d goal_position = play_state.ball_placement_point().value();
        Eigen::Vector2d ball_position = world_state.ball.position;

        Eigen::Vector2d delta_pg = robot_position - goal_position;
        A(kBallPlacementConstraintIdx, 0) = -delta_pg(0);
        A(kBallPlacementConstraintIdx, 1) = - delta_pg(1);

        if (delta_pg.norm() < kSafeDistanceBall + play_state.max_speed() * dt_) {
            double h_g = delta_pg.norm() - kSafeDistanceBall;
            u(kBallPlacementConstraintIdx) = delta_pg.norm() * control_gain_ * h_g;
        } else {
            u(kBallPlacementConstraintIdx) = 1e9;
        }
        l(kBallPlacementConstraintIdx) = -1e9;

        // Calculate Ball Placement Line Barriers
        Eigen::Vector2d d = goal_position - ball_position; //NOLINT(readability-identifier-length)
        Eigen::Vector2d n1(-d.y(), d.x()); //NOLINT(readability-identifier-length)
        n1.normalize();
        Eigen::Vector2d n2 = -n1; //NOLINT(readability-identifier-length)
        Eigen::Vector2d n = (n1.dot(robot_position - ball_position) > 0) ? n1 : n2; //NOLINT(readability-identifier-length)
        A(kBallPlacementConstraintIdx + 1, 0) = n(0);
        A(kBallPlacementConstraintIdx + 1, 1) = n(1);

        Eigen::Vector2d projection = robot_position.dot(d) / d.squaredNorm() * d;
        double distance_from_line = n.transpose() * (robot_position - ball_position);
        if (
            distance_from_line <= kSafeDistanceBall + play_state.max_speed() * dt_ &&
            min(goal_position.x(), ball_position.x()) <= projection.x() && projection.x() <= max(goal_position.x(), ball_position.x()) &&
            min(goal_position.y(), ball_position.y()) <= projection.y() && projection.y() <= max(goal_position.y(), ball_position.y())
        ) {
            u(kBallPlacementConstraintIdx + 1) = distance_from_line - kSafeDistanceBall;
        } else {
            u(kBallPlacementConstraintIdx + 1) = 1e9;
        }
        l(kBallPlacementConstraintIdx + 1) = -1e9;
    } else {
        // For solver optimization, we still need A to have nonzero values
        // at 2N+8 and 2N+9
        A(kBallPlacementConstraintIdx, 0) = -1.0;
        A(kBallPlacementConstraintIdx, 1) = -1.0;
        u(kBallPlacementConstraintIdx) = 1e9;
        l(kBallPlacementConstraintIdx) = -1e9;
        A(kBallPlacementConstraintIdx + 1, 0) = -1.0;
        A(kBallPlacementConstraintIdx + 1, 1) = -1.0;
        u(kBallPlacementConstraintIdx + 1) = 1e9;
        l(kBallPlacementConstraintIdx + 1) = -1e9;
    }
}

void LinearCBF::calculate_velocity_constraints(
    const PlayState& play_state,
    Eigen::Matrix<double, kConstraints, 2>& A, //NOLINT(readability-identifier-naming, readability-identifier-length)
    Eigen::Matrix<double, kConstraints, 1>& l, //NOLINT(readability-identifier-naming, readability-identifier-length, bugprone-easily-swappable-parameters)
    Eigen::Matrix<double, kConstraints, 1>& u //NOLINT(readability-identifier-naming, readability-identifier-length)
) {
    double max_speed = play_state.max_speed();

    // abs(v_x) <= max_speed
    A(kVelocityConstraintIdx, 0) = 1;
    A(kVelocityConstraintIdx, 1) = 0;
    l(kVelocityConstraintIdx) = -max_speed;
    u(kVelocityConstraintIdx) = max_speed;

    // absS(v_y) <= max_speed
    A(kVelocityConstraintIdx + 1, 0) = 0;
    A(kVelocityConstraintIdx + 1, 1) = 1;
    l(kVelocityConstraintIdx + 1) = -max_speed;
    u(kVelocityConstraintIdx + 1) = max_speed;

    // 1/sqrt(2) * (v_x + v_y) <= max_speed
    A(kVelocityConstraintIdx + 2, 0) = 1 / sqrt(2);
    A(kVelocityConstraintIdx + 2, 1) = 1 / sqrt(2);
    l(kVelocityConstraintIdx + 2) = -max_speed;
    u(kVelocityConstraintIdx + 2) = max_speed;

    // 1 / sqrt(2) * (v_x - v_y) <= max_speed
    A(kVelocityConstraintIdx + 3, 0) = 1 / sqrt(2);
    A(kVelocityConstraintIdx + 3, 1) = -1 / sqrt(2);
    l(kVelocityConstraintIdx + 3) = -max_speed;
    u(kVelocityConstraintIdx + 3) = max_speed;
}

} // namespace control