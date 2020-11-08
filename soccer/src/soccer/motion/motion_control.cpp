#include "motion_control.hpp"

#include <optional>

#include <context.hpp>
#include <rj_common/utils.hpp>
#include <rj_geometry/util.hpp>
#include <rj_utils/logging.hpp>
#include <robot_config.hpp>

#include "planning/instant.hpp"

using namespace std;
using namespace rj_geometry;
using namespace Planning;

#pragma mark Config Variables

REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble* MotionControl::max_acceleration;
ConfigDouble* MotionControl::max_velocity;
ConfigDouble* MotionControl::x_multiplier;

void MotionControl::create_configuration(Configuration* cfg) {
    max_acceleration = new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
    x_multiplier = new ConfigDouble(cfg, "MotionControl/X_Multiplier", 1.0);
}

#pragma mark MotionControl

MotionControl::MotionControl(Context* context, int shell_id)
    : shell_id_(shell_id),
      angle_controller_(0, 0, 0, 50, 0),
      drawer_(&context->debug_drawer),
      config_(context->robot_config.get()) {}

void MotionControl::run(const RobotState& state, const Planning::Trajectory& trajectory,
                        bool is_joystick_controlled, MotionSetpoint* setpoint) {
    // If we don't have a setpoint (output velocities) or we're under joystick
    // control, reset our PID controllers and exit (but don't force a stop).
    if ((setpoint == nullptr) || is_joystick_controlled) {
        reset();
        return;
    }

    if (!state.visible || trajectory.empty()) {
        stop(setpoint);
        return;
    }

    update_params();

    // We run this at 60Hz, so we want to do motion control off of the goal
    // position for the next frame. Evaluate the trajectory there.
    RJ::Seconds dt(1.0 / 60);
    RJ::Time eval_time = RJ::now() + dt;

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

    // TODO: Calculate acceleration and use it to improve response.
    // TODO: Clamp acceleration

    Twist correction = Twist::zero();
    if (maybe_pose_target) {
        Pose error = maybe_pose_target.value() - state.pose;
        error.heading() = fix_angle_radians(error.heading());
        correction = Twist(position_x_controller_.run(error.position().x()),
                           position_y_controller_.run(error.position().y()),
                           angle_controller_.run(error.heading()));
    } else {
        reset();
    }

    // Apply the correction and rotate into the world frame.
    Twist result_world = velocity_target + correction;
    Twist result_body(result_world.linear().rotated(M_PI_2 - state.pose.heading()),
                      result_world.angular());

    // Use default constraints. Planning should be in charge of enforcing
    // constraints on the trajectory, here we just follow it.
    // TODO(#1500): Use this robot's constraints here.
    RobotConstraints constraints;

    if (result_body.linear().mag() > constraints.mot.max_speed) {
        result_body.linear() *= constraints.mot.max_speed / result_body.linear().mag();
    }

    result_body.angular() =
        std::clamp(result_body.angular(), -constraints.rot.max_speed, constraints.rot.max_speed);

    set_velocity(setpoint, result_body);

    // Debug drawing
    {
        if (at_end) {
            drawer_->draw_circle(maybe_target->pose.position(), .15, Qt::red, "Planning");
        } else if (maybe_target) {
            drawer_->draw_circle(maybe_target->pose.position(), .15, Qt::green, "Planning");
        }

        // Line for velocity when we have a target
        if (maybe_pose_target) {
            Pose pose_target = maybe_pose_target.value();
            drawer_->draw_line(pose_target.position(),
                               pose_target.position() + result_world.linear(), Qt::blue,
                               "MotionControl");
        }
    }
}

void MotionControl::reset() {
    position_x_controller_.clearWindup();
    position_y_controller_.clearWindup();
    angle_controller_.clearWindup();
}

void MotionControl::set_velocity(MotionSetpoint* setpoint, Twist target_vel) {
    // Limit Velocity
    target_vel.linear().clamp(*max_velocity);

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
    // update PID parameters
    position_x_controller_.kp = *config_->translation.p;
    position_x_controller_.ki = *config_->translation.i;
    position_x_controller_.setWindup(*config_->translation.i_windup);
    position_x_controller_.kd = *config_->translation.d;
    position_y_controller_.kp = *config_->translation.p;
    position_y_controller_.ki = *config_->translation.i;
    position_y_controller_.setWindup(*config_->translation.i_windup);
    position_y_controller_.kd = *config_->translation.d;
    angle_controller_.kp = *config_->rotation.p;
    angle_controller_.ki = *config_->rotation.i;
    angle_controller_.kd = *config_->rotation.d;
}

void MotionControl::reset_pid_controllers() {
    position_x_controller_.reset();
    position_y_controller_.reset();
    angle_controller_.reset();
}

void MotionControl::stop(MotionSetpoint* setpoint) {
    setpoint->clear();
    reset_pid_controllers();
}
