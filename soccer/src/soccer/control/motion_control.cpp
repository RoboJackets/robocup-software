#include "motion_control.hpp"

#include <optional>

#include <context.hpp>
#include <rj_common/utils.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/logging.hpp>

#include "game_state.hpp"
#include "planning/instant.hpp"

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
