#include "MotionControl.hpp"

#include <optional>

#include <Geometry2d/Util.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include <Utils.hpp>
#include <planning/MotionInstant.hpp>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

#pragma mark Config Variables

REGISTER_CONFIGURABLE(MotionControl);

ConfigDouble* MotionControl::_max_acceleration;
ConfigDouble* MotionControl::_max_velocity;
ConfigDouble* MotionControl::_x_multiplier;

void MotionControl::createConfiguration(Configuration* cfg) {
    _max_acceleration =
        new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    _max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
    _x_multiplier = new ConfigDouble(cfg, "MotionControl/X_Multiplier", 1.0);
}

#pragma mark MotionControl

MotionControl::MotionControl(Context* context, int shell_id)
    : _shell_id(shell_id),
      _angleController(0, 0, 0, 50, 0),
      _drawer(&context->debug_drawer),
      _config(context->robot_config.get()) {}

void MotionControl::run(const RobotState& state,
                        const Planning::AngleFunctionPath& path,
                        bool is_joystick_controlled, MotionSetpoint* setpoint) {
    // If we don't have a setpoint (output velocities) or we're under joystick
    // control, reset our PID controllers and exit (but don't force a stop).
    if (!setpoint || is_joystick_controlled) {
        reset();
        return;
    }

    if (!state.visible || !path.path) {
        stop(setpoint);
        return;
    }

    updateParams();

    // We run this at 60Hz, so we want to do motion control off of the goal
    // position for the next frame. Evaluate the path there.
    RJ::Seconds dt(1.0 / 60);
    RJ::Seconds time_into_path = (RJ::now() - path.startTime()) + dt;

    std::optional<RobotInstant> maybe_target = path.evaluate(time_into_path);

    // If we're past the end of the path, do motion control off of the end.
    bool at_end = time_into_path > path.getDuration();
    if (at_end) {
        maybe_target = path.end();
    }

    std::optional<Pose> maybe_pose_target;
    Twist velocity_target = Twist::Zero();

    // Set up goals from our target motion instant.
    if (maybe_target) {
        auto target = maybe_target.value();
        maybe_pose_target = target.pose();
        velocity_target = target.twist();

        // If we have no goal angle, set it to the current angle.
        if (!(target.angle && target.angle.value().angle)) {
            maybe_pose_target.value().heading() = state.pose.heading();
        }
    }

    // TODO: Calculate acceleration and use it to improve response.
    // TODO: Clamp acceleration

    Twist correction = Twist::Zero();
    if (maybe_pose_target) {
        Pose error = maybe_pose_target.value() - state.pose;
        error.heading() = fixAngleRadians(error.heading());
        correction = Twist(_positionXController.run(error.position().x()),
                           _positionYController.run(error.position().y()),
                           _angleController.run(error.heading()));
    } else {
        reset();
    }

    // Apply the correction and rotate into the world frame.
    Twist result_world = velocity_target + correction;
    Twist result_body(
        result_world.linear().rotated(M_PI_2 - state.pose.heading()),
        result_world.angular());

    // Use default constraints. Planning should be in charge of enforcing
    // constraints on the path, here we just follow it.
    // TODO(Kyle): Use this robot's constraints here.
    RobotConstraints constraints;

    if (result_body.linear().mag() > constraints.mot.maxSpeed) {
        result_body.linear() *=
            constraints.mot.maxSpeed / result_body.linear().mag();
    }

    result_body.angular() =
        std::clamp(result_body.angular(), -constraints.rot.maxSpeed,
                   constraints.rot.maxSpeed);

    setVelocity(setpoint, result_body);

    // Debug drawing
    {
        if (at_end) {
            _drawer->drawCircle(maybe_target->motion.pos, .15, Qt::red,
                                "Planning");
        } else if (maybe_target) {
            _drawer->drawCircle(maybe_target->motion.pos, .15, Qt::green,
                                "Planning");
        }

        // Line for velocity when we have a target
        if (maybe_pose_target) {
            Pose pose_target = maybe_pose_target.value();
            _drawer->drawLine(pose_target.position(),
                              pose_target.position() + result_world.linear(),
                              Qt::blue, "MotionControl");
        }
    }
}

void MotionControl::reset() {
    _positionXController.clearWindup();
    _positionYController.clearWindup();
    _angleController.clearWindup();
}

void MotionControl::setVelocity(MotionSetpoint* setpoint, Twist target_vel) {
    // Limit Velocity
    target_vel.linear().clamp(*_max_velocity);

    // make sure we don't send any bad values
    if (Eigen::Vector3d(target_vel).hasNaN()) {
        target_vel = Twist::Zero();
        debugThrow("A bad value was calculated.");
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

void MotionControl::updateParams() {
    // update PID parameters
    _positionXController.kp = *_config->translation.p;
    _positionXController.ki = *_config->translation.i;
    _positionXController.setWindup(*_config->translation.i_windup);
    _positionXController.kd = *_config->translation.d;
    _positionYController.kp = *_config->translation.p;
    _positionYController.ki = *_config->translation.i;
    _positionYController.setWindup(*_config->translation.i_windup);
    _positionYController.kd = *_config->translation.d;
    _angleController.kp = *_config->rotation.p;
    _angleController.ki = *_config->rotation.i;
    _angleController.kd = *_config->rotation.d;
}

void MotionControl::resetPIDControllers() {
    _positionXController.reset();
    _positionYController.reset();
    _angleController.reset();
}

void MotionControl::stop(MotionSetpoint* setpoint) {
    setpoint->clear();
    resetPIDControllers();
}
