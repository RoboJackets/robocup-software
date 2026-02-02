#include "rj_control/motion_control.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace control {

using planning::RobotInstant;
using rj_geometry::Pose;
using rj_geometry::Twist;

DEFINE_FLOAT64(params::kMotionControlParamModule, max_acceleration, 3.0,
               "Maximum acceleration limit (motion control) (m/s^2)");
DEFINE_FLOAT64(params::kMotionControlParamModule, max_velocity, 2.4,
               "Maximum velocity limit (motion control) (m/s)");
DEFINE_FLOAT64(params::kMotionControlParamModule, max_angular_velocity, 5.0,
               "Maximum angular velocity limit (motion control) (rad/s)");
DEFINE_FLOAT64(params::kMotionControlParamModule, rotation_kp, 10.0,
               "Kp for rotation ((rad/s)/rad)");
DEFINE_FLOAT64(params::kMotionControlParamModule, rotation_ki, 0.0,
               "Ki for rotation ((rad/s)/(rad*s))");
DEFINE_FLOAT64(params::kMotionControlParamModule, rotation_kd, 0.0,
               "Kd for rotation ((rad/s)/(rad/s))");
DEFINE_INT64(params::kMotionControlParamModule, rotation_windup, 0,
             "Windup limit for rotation (unknown units)");
DEFINE_FLOAT64(params::kMotionControlParamModule, translation_kp, 0.6,
               "Kp for translation ((m/s)/m)");
DEFINE_FLOAT64(params::kMotionControlParamModule, translation_ki, 0.0,
               "Ki for translation ((m/s)/(m*s))");
DEFINE_FLOAT64(params::kMotionControlParamModule, translation_kd, 0.3,
               "Kd for translation ((m/s)/(m/s))");
DEFINE_INT64(params::kMotionControlParamModule, translation_windup, 0,
             "Windup limit for translation (unknown units)");
DEFINE_FLOAT64(params::kMotionControlParamModule, path_lookahead_distance, 0.1,
               "Lookahead distance along the trajectory for PID error (m)");

namespace {

struct PathProjection {
    rj_geometry::Point closest_point;
    double s_closest = 0.0;
    double total_length = 0.0;
    bool valid = false;
};

double trajectory_length(const planning::Trajectory& trajectory) {
    const auto& instants = trajectory.instants();
    if (instants.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (size_t i = 0; i + 1 < instants.size(); ++i) {
        length += (instants[i + 1].pose.position() - instants[i].pose.position()).mag();
    }
    return length;
}

PathProjection project_to_trajectory_forward(const planning::Trajectory& trajectory,
                                             const rj_geometry::Point& query, double min_s) {
    PathProjection result;
    const auto& instants = trajectory.instants();
    if (instants.size() < 2) {
        return result;
    }

    double best_dist_sq = std::numeric_limits<double>::infinity();
    double length_so_far = 0.0;
    const double kEps = 1e-6;

    for (size_t i = 0; i + 1 < instants.size(); ++i) {
        rj_geometry::Point p0 = instants[i].pose.position();
        rj_geometry::Point p1 = instants[i + 1].pose.position();
        rj_geometry::Point delta = p1 - p0;
        double seg_len_sq = delta.magsq();
        double seg_len = 0.0;
        double t = 0.0;
        rj_geometry::Point nearest = p0;

        if (seg_len_sq > 1e-12) {
            seg_len = std::sqrt(seg_len_sq);
            double t_proj = (query - p0).dot(delta) / seg_len_sq;
            // Enforce forward progress by clamping to the portion of the segment beyond min_s.
            double seg_start_s = length_so_far;
            double seg_end_s = length_so_far + seg_len;

            if (seg_end_s + kEps < min_s) {
                length_so_far += seg_len;
                continue;  // Entire segment is behind min_s.
            }

            double t_min = 0.0;
            if (min_s > seg_start_s) {
                t_min = (min_s - seg_start_s) / seg_len;
            }

            t = std::clamp(t_proj, t_min, 1.0);
            nearest = p0 + delta * t;
        }

        double dist_sq = (query - nearest).magsq();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            result.closest_point = nearest;
            result.s_closest = length_so_far + t * seg_len;
            result.valid = true;
        }

        length_so_far += seg_len;
    }

    result.total_length = length_so_far;
    return result;
}

rj_geometry::Point point_at_distance(const planning::Trajectory& trajectory, double distance) {
    const auto& instants = trajectory.instants();
    if (instants.empty()) {
        return rj_geometry::Point();
    }
    if (instants.size() == 1 || distance <= 0.0) {
        return instants.front().pose.position();
    }

    double remaining = distance;
    for (size_t i = 0; i + 1 < instants.size(); ++i) {
        rj_geometry::Point p0 = instants[i].pose.position();
        rj_geometry::Point p1 = instants[i + 1].pose.position();
        double seg_len = (p1 - p0).mag();
        if (seg_len <= 1e-12) {
            continue;
        }
        if (remaining <= seg_len) {
            double t = remaining / seg_len;
            return p0 + (p1 - p0) * t;
        }
        remaining -= seg_len;
    }

    return instants.back().pose.position();
}

rj_geometry::Point tangent_at_distance(const planning::Trajectory& trajectory, double distance,
                                       rj_geometry::Point* out_unit_dir, double* out_seg_len) {
    const auto& instants = trajectory.instants();
    rj_geometry::Point last_dir{1, 0};
    if (instants.size() < 2) {
        if (out_unit_dir) {
            *out_unit_dir = last_dir;
        }
        if (out_seg_len) {
            *out_seg_len = 0.0;
        }
        return instants.empty() ? rj_geometry::Point() : instants.front().pose.position();
    }

    double remaining = distance;
    for (size_t i = 0; i + 1 < instants.size(); ++i) {
        rj_geometry::Point p0 = instants[i].pose.position();
        rj_geometry::Point p1 = instants[i + 1].pose.position();
        rj_geometry::Point delta = p1 - p0;
        double seg_len = delta.mag();
        if (seg_len <= 1e-12) {
            continue;
        }
        last_dir = delta / seg_len;

        if (remaining <= seg_len) {
            double t = remaining / seg_len;
            if (out_unit_dir) {
                *out_unit_dir = last_dir;
            }
            if (out_seg_len) {
                *out_seg_len = seg_len;
            }
            return p0 + delta * t;
        }
        remaining -= seg_len;
    }

    if (out_unit_dir) {
        *out_unit_dir = last_dir;
    }
    if (out_seg_len) {
        *out_seg_len = 0.0;
    }
    return instants.back().pose.position();
}

bool tangent_speed_at_distance(const planning::Trajectory& trajectory, double distance,
                               rj_geometry::Point* out_dir, double* out_speed) {
    const auto& instants = trajectory.instants();
    if (instants.size() < 2) {
        return false;
    }

    double remaining = distance;
    rj_geometry::Point dir{1, 0};

    for (size_t i = 0; i + 1 < instants.size(); ++i) {
        rj_geometry::Point p0 = instants[i].pose.position();
        rj_geometry::Point p1 = instants[i + 1].pose.position();
        rj_geometry::Point delta = p1 - p0;
        double seg_len = delta.mag();
        if (seg_len <= 1e-12) {
            continue;
        }
        dir = delta / seg_len;

        double speed0 = instants[i].velocity.linear().mag();
        double speed1 = instants[i + 1].velocity.linear().mag();

        if (remaining <= seg_len) {
            double t = remaining / seg_len;
            double speed = speed0 + (speed1 - speed0) * t;
            if (out_dir) {
                *out_dir = dir;
            }
            if (out_speed) {
                *out_speed = speed;
            }
            return true;
        }
        remaining -= seg_len;
    }

    // If distance is beyond total length, use last segment values.
    if (out_dir) {
        *out_dir = dir;
    }
    if (out_speed) {
        *out_speed = instants.back().velocity.linear().mag();
    }
    return true;
}

}  // namespace

MotionControl::MotionControl(int shell_id, rclcpp::Node* node)
    : shell_id_(shell_id),
      angle_controller_(0, 0, 0, 50, 0),
      drawer_(
          node->create_publisher<rj_drawing_msgs::msg::DebugDraw>(viz::topics::kDebugDrawTopic, 10),
          fmt::format("motion_control/{}", std::to_string(shell_id))) {
    motion_setpoint_pub_ = node->create_publisher<MotionSetpoint::Msg>(
        topics::motion_setpoint_topic(shell_id_), rclcpp::QoS(1));
    target_state_pub_ = node->create_publisher<RobotState::Msg>(
        topics::desired_state_topic(shell_id_), rclcpp::QoS(1));
    // Update motion control triggered on world state publish.
    trajectory_sub_ = node->create_subscription<planning::Trajectory::Msg>(
        planning::topics::trajectory_topic(shell_id), rclcpp::QoS(1),
        [this](planning::Trajectory::Msg::SharedPtr trajectory) {  // NOLINT
            trajectory_ = rj_convert::convert_from_ros(*trajectory);
        });
    world_state_sub_ = node->create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](WorldState::Msg::SharedPtr world_state_msg) {  // NOLINT
            RobotState state =
                rj_convert::convert_from_ros(world_state_msg->our_robots.at(shell_id_));

            // TODO(Kyle): Handle the joystick-controlled case here. In the long run we want to
            // convert this to an action. Should we do that now?
            // Note: The Motion Control Node is not spawned when manual control is active
            bool is_joystick_controlled = false;
            MotionSetpoint setpoint;
            run(state, trajectory_, play_state_, is_joystick_controlled, &setpoint);
            motion_setpoint_pub_->publish(rj_convert::convert_to_ros(setpoint));
        });
    play_state_sub_ = node->create_subscription<PlayState::Msg>(
        referee::topics::kPlayStateTopic, rclcpp::QoS(1).transient_local(),
        [this](PlayState::Msg::SharedPtr play_state_msg) {  // NOLINT
            play_state_ = rj_convert::convert_from_ros(*play_state_msg).state();
        });

    error_x_pub_ =
        node->create_publisher<std_msgs::msg::Float64>("debug/motion_control/pose_error_x", 10);
    error_y_pub_ =
        node->create_publisher<std_msgs::msg::Float64>("debug/motion_control/pose_error_y", 10);
    error_heading_pub_ = node->create_publisher<std_msgs::msg::Float64>(
        "debug/motion_control/pose_error_heading", 10);
}

void MotionControl::run(const RobotState& state, const planning::Trajectory& trajectory,
                        const PlayState::State& play_state, bool is_joystick_controlled,
                        MotionSetpoint* setpoint) {
    // If we don't have a setpoint (output velocities) or we're under joystick
    // control, reset our PID controllers and exit (but don't force a stop).
    if ((setpoint == nullptr) || is_joystick_controlled) {
        reset();
        return;
    }

    if (!state.visible || trajectory.empty() || play_state == PlayState::State::Halt) {
        stop(setpoint);
        return;
    }

    // Reset progress tracking on new trajectories.
    if (!last_progress_valid_ || trajectory.begin_time() != last_trajectory_begin_time_) {
        last_path_progress_ = 0.0;
        last_progress_valid_ = true;
        last_trajectory_begin_time_ = trajectory.begin_time();
    }

    update_params();

    // We run this at 60Hz, so we want to do motion control off of the goal
    // position for the next frame. Evaluate the trajectory there.
    RJ::Seconds dt(1.0 / 60);
    RJ::Time eval_time = state.timestamp + dt;

    double total_traj_length = trajectory_length(trajectory);

    std::optional<RobotInstant> maybe_target = trajectory.evaluate(eval_time);
    bool at_end = eval_time > trajectory.end_time();

    // If we're past the end of the trajectory, do motion control off of the
    // end.
    if (at_end) {
        maybe_target = trajectory.last();
    }

    std::optional<Pose> maybe_pose_target;
    Twist velocity_target = Twist::zero();
    std::optional<rj_geometry::Point> maybe_closest_point;
    std::optional<rj_geometry::Point> maybe_lookahead_point;
    rj_geometry::Point feedforward_linear = rj_geometry::Point{0, 0};
    double feedforward_angular = 0.0;

    // Set up goals from our target motion instant.
    if (maybe_target) {
        auto target = maybe_target.value();
        maybe_pose_target = target.pose;
        feedforward_angular = target.velocity.angular();

        PathProjection projection =
            project_to_trajectory_forward(trajectory, state.pose.position(),
                                          std::min(last_path_progress_, total_traj_length));
        if (projection.valid && projection.total_length > 0.0) {
            double lookahead_distance = std::max(0.0, PARAM_path_lookahead_distance);
            double min_forward_slack = 0.02;  // meters of allowed backward tolerance
            double min_s = std::max(0.0, std::min(projection.s_closest, projection.total_length));
            min_s = std::min(min_s + min_forward_slack, projection.total_length);
            double s_target =
                std::clamp(projection.s_closest + lookahead_distance, min_s, projection.total_length);
            rj_geometry::Point lookahead_point = point_at_distance(trajectory, s_target);
            maybe_closest_point = projection.closest_point;
            maybe_lookahead_point = lookahead_point;
            maybe_pose_target->position() = lookahead_point;
            last_path_progress_ = projection.s_closest;

            rj_geometry::Point dir;
            double speed = 0.0;
            if (tangent_speed_at_distance(trajectory, s_target, &dir, &speed) && dir.mag() > 1e-6) {
                feedforward_linear = dir * speed;
            } else {
                feedforward_linear = target.velocity.linear();
            }
        } else {
            feedforward_linear = target.velocity.linear();
        }
        velocity_target = Twist(feedforward_linear, feedforward_angular);
    }

    // TODO(Kyle): Calculate acceleration and use it to improve response.
    // TODO(Kyle): Clamp acceleration

    Twist correction = Twist::zero();

    if (maybe_pose_target) {
        Pose error = maybe_pose_target.value() - state.pose;
        error.heading() = fix_angle_radians(error.heading());

        if (error_x_pub_->get_subscription_count() > 0) {
            std_msgs::msg::Float64 error_x_msg;
            error_x_msg.data = error.position().x();
            error_x_pub_->publish(error_x_msg);
        }

        if (error_y_pub_->get_subscription_count() > 0) {
            std_msgs::msg::Float64 error_y_msg;
            error_y_msg.data = error.position().y();
            error_y_pub_->publish(error_y_msg);
        }

        if (error_heading_pub_->get_subscription_count() > 0) {
            std_msgs::msg::Float64 error_heading_msg;
            error_heading_msg.data = error.heading();
            error_heading_pub_->publish(error_heading_msg);
        }

        correction = Twist(position_x_controller_.run(static_cast<float>(error.position().x())),
                           position_y_controller_.run(static_cast<float>(error.position().y())),
                           angle_controller_.run(static_cast<float>(error.heading())));
    } else {
        reset();
    }

    // Apply the correction and rotate into the world frame.
    Twist result_world = velocity_target + correction;
    Twist result_body(result_world.linear().rotated(M_PI_2 - state.pose.heading()),
                      result_world.angular());

    set_velocity(setpoint, result_body);

    {
        // Debug drawing
        using rj_geometry::Circle;
        using rj_geometry::Segment;
        if (maybe_lookahead_point) {
            drawer_.draw_circle(Circle(maybe_lookahead_point.value(), .15),
                                at_end ? QColor(255, 0, 0, 0) : QColor(0, 255, 0, 0));
        } else if (maybe_target) {
            drawer_.draw_circle(Circle(maybe_target->pose.position(), .15),
                                at_end ? QColor(255, 0, 0, 0) : QColor(0, 255, 0, 0));
        }

        if (maybe_closest_point) {
            drawer_.draw_circle(Circle(maybe_closest_point.value(), .10), QColor(255, 255, 0, 0));
        }

        // Line for velocity when we have a target
        if (maybe_pose_target) {
            Pose pose_target = maybe_pose_target.value();
            drawer_.draw_segment(
                Segment(pose_target.position(), pose_target.position() + result_world.linear()),
                Qt::blue);
        }

        drawer_.publish();
    }

    if (maybe_target) {
        RobotState desired_state;
        if (maybe_pose_target) {
            desired_state.pose = *maybe_pose_target;
        } else {
            desired_state.pose = maybe_target->pose;
        }
        desired_state.velocity = velocity_target;
        desired_state.timestamp = maybe_target->stamp;
        desired_state.visible = true;
        target_state_pub_->publish(rj_convert::convert_to_ros(desired_state));
    }
}

void MotionControl::set_velocity(MotionSetpoint* setpoint, Twist target_vel) {
    // Limit Velocity
    target_vel.linear().clamp(PARAM_max_velocity);
    target_vel.angular() =
        std::clamp(target_vel.angular(), -PARAM_max_angular_velocity, PARAM_max_angular_velocity);

    // make sure we don't send any bad values
    if (Eigen::Vector3d(target_vel).hasNaN()) {
        target_vel = Twist::zero();
        rj_utils::debug_throw("A bad value was calculated.");
    }

    // Note: we used to set minimum effective speeds here. However, that should
    // really be handled in motion control, because it's just a hack to
    // compensate for static friction effects.
    // It messes up precise shots, so it's been removed.

    // set control values
    setpoint->xvelocity = target_vel.linear().x();
    setpoint->yvelocity = target_vel.linear().y();
    setpoint->avelocity = target_vel.angular();
}

void MotionControl::update_params() {
    // Update PID parameters
    position_x_controller_.kp = static_cast<float>(PARAM_translation_kp);
    position_x_controller_.ki = static_cast<float>(PARAM_translation_ki);
    position_x_controller_.kd = static_cast<float>(PARAM_translation_kd);
    position_x_controller_.setWindup(PARAM_translation_windup);

    position_y_controller_.kp = static_cast<float>(PARAM_translation_kp);
    position_y_controller_.ki = static_cast<float>(PARAM_translation_ki);
    position_y_controller_.kd = static_cast<float>(PARAM_translation_kd);
    position_y_controller_.setWindup(PARAM_translation_windup);

    angle_controller_.kp = static_cast<float>(PARAM_rotation_kp);
    angle_controller_.ki = static_cast<float>(PARAM_rotation_ki);
    angle_controller_.kd = static_cast<float>(PARAM_rotation_kd);
    angle_controller_.setWindup(PARAM_rotation_windup);
}

void MotionControl::reset() {
    position_x_controller_.reset();
    position_y_controller_.reset();
    angle_controller_.reset();
}

void MotionControl::stop(MotionSetpoint* setpoint) {
    *setpoint = {};
    reset();
}

}  // namespace control
