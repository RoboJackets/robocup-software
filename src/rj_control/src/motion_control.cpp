#include "rj_control/motion_control.hpp"

namespace control {

using planning::RobotInstant;
using rj_geometry::Pose;
using rj_geometry::Twist;

MotionControl::MotionControl(int shell_id, rclcpp::Node* node)
        : shell_id_(shell_id),
            node_(node),
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
        planning::topics::trajectory_topic(shell_id_), rclcpp::QoS(1),
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
    // Initialize params from node
    read_params();
}

// Read per-robot parameters from the node (allows robots.<id>.<param> YAML)
// This sets the defaults and reads overrides from the node.
void MotionControl::read_params() {
    const std::string root = "robots." + std::to_string(shell_id_) + ".";

    auto declare_or_get = [this, &root](const std::string& name, auto default_val) {
        using T = std::decay_t<decltype(default_val)>;
        const std::string full = root + name;
        T value{};
        if (node_->has_parameter(full)) {
            node_->get_parameter(full, value);
            return value;
        }
        // fallback to global param name
        if (node_->has_parameter(name)) {
            node_->get_parameter(name, value);
            return value;
        }
        // No parameter set in node: return default (do not declare to avoid conflicts)
        return default_val;
    };

    params_.max_acceleration = declare_or_get("max_acceleration", params_.max_acceleration);
    params_.max_velocity = declare_or_get("max_velocity", params_.max_velocity);
    params_.max_angular_velocity =
        declare_or_get("max_angular_velocity", params_.max_angular_velocity);

    params_.rotation_kp = declare_or_get("rotation_kp", params_.rotation_kp);
    params_.rotation_ki = declare_or_get("rotation_ki", params_.rotation_ki);
    params_.rotation_kd = declare_or_get("rotation_kd", params_.rotation_kd);
    params_.rotation_windup = declare_or_get("rotation_windup", params_.rotation_windup);

    params_.translation_kp = declare_or_get("translation_kp", params_.translation_kp);
    params_.translation_ki = declare_or_get("translation_ki", params_.translation_ki);
    params_.translation_kd = declare_or_get("translation_kd", params_.translation_kd);
    params_.translation_windup = declare_or_get("translation_windup", params_.translation_windup);
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

    update_params();

    // We run this at 60Hz, so we want to do motion control off of the goal
    // position for the next frame. Evaluate the trajectory there.
    RJ::Seconds dt(1.0 / 60);
    RJ::Time eval_time = state.timestamp + dt;

    std::optional<RobotInstant> maybe_target = trajectory.evaluate(eval_time);
    bool at_end = eval_time > trajectory.end_time();

    // If we're past the end of the trajectory, do motion control off of the
    // end.
    if (at_end) {
        maybe_target = trajectory.last();
    }

    std::optional<Pose> maybe_pose_target;
    Twist velocity_target = Twist::zero();

    // Set up goals from our target motion instant.
    if (maybe_target) {
        auto target = maybe_target.value();
        maybe_pose_target = target.pose;
        velocity_target = target.velocity;
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
        if (at_end) {
            drawer_.draw_circle(Circle(maybe_target->pose.position(), .15), QColor(255, 0, 0, 0));
        } else if (maybe_target) {
            drawer_.draw_circle(Circle(maybe_target->pose.position(), .15), QColor(0, 255, 0, 0));
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
        desired_state.pose = maybe_target->pose;
        desired_state.velocity = velocity_target;
        desired_state.timestamp = maybe_target->stamp;
        desired_state.visible = true;
        target_state_pub_->publish(rj_convert::convert_to_ros(desired_state));
    }
}

void MotionControl::set_velocity(MotionSetpoint* setpoint, Twist target_vel) {
    // Limit Velocity
    target_vel.linear().clamp(params_.max_velocity);
    target_vel.angular() =
        std::clamp(target_vel.angular(), -params_.max_angular_velocity, params_.max_angular_velocity);

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
    // Re-read params (allow dynamic overrides) and update PID parameters
    if (node_) {
        read_params();
    }
    position_x_controller_.kp = static_cast<float>(params_.translation_kp);
    position_x_controller_.ki = static_cast<float>(params_.translation_ki);
    position_x_controller_.kd = static_cast<float>(params_.translation_kd);
    position_x_controller_.setWindup(params_.translation_windup);

    position_y_controller_.kp = static_cast<float>(params_.translation_kp);
    position_y_controller_.ki = static_cast<float>(params_.translation_ki);
    position_y_controller_.kd = static_cast<float>(params_.translation_kd);
    position_y_controller_.setWindup(params_.translation_windup);

    angle_controller_.kp = static_cast<float>(params_.rotation_kp);
    angle_controller_.ki = static_cast<float>(params_.rotation_ki);
    angle_controller_.kd = static_cast<float>(params_.rotation_kd);
    angle_controller_.setWindup(params_.rotation_windup);
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
