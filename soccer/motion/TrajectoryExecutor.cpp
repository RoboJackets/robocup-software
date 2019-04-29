#include "TrajectoryExecutor.hpp"
#include "Robot.hpp"

REGISTER_CONFIGURABLE(TrajectoryExecutor);

ConfigDouble* TrajectoryExecutor::max_acceleration;
ConfigDouble* TrajectoryExecutor::max_velocity;

void TrajectoryExecutor::createConfiguration(Configuration* cfg) {
    max_acceleration =
        new ConfigDouble(cfg, "MotionControl/Max Acceleration", 1.5);
    max_velocity = new ConfigDouble(cfg, "MotionControl/Max Velocity", 2.0);
}

TrajectoryExecutor::TrajectoryExecutor(OurRobot* robot) : robot(robot) {
    robot->robotPacket.set_uid(robot->shell());
}

void TrajectoryExecutor::stop() {
    if (!robot) return;

    // We still need to give the robot vision data
    robot->control->set_vision_pose_x(robot->pos.x());
    robot->control->set_vision_pose_y(robot->pos.y());
    robot->control->set_vision_pose_theta(robot->angle);

    // But we can tell it to go at zero velocity (and hold its position)
    robot->control->set_goal_pose_x(robot->pos.x());
    robot->control->set_goal_pose_y(robot->pos.y());
    robot->control->set_goal_pose_theta(robot->angle);

    robot->control->set_goal_velocity_x(0);
    robot->control->set_goal_velocity_y(0);
    robot->control->set_goal_velocity_theta(0);

    robot->control->set_goal_acceleration_x(0);
    robot->control->set_goal_acceleration_y(0);
    robot->control->set_goal_acceleration_theta(0);

    robot->control->set_motion_mode(Packet::Control::VELOCITY_CONTROL);
}

void TrajectoryExecutor::run() {
    if (!robot) return;

    robot->control->set_vision_pose_x(robot->pos.x());
    robot->control->set_vision_pose_y(robot->pos.y());
    robot->control->set_vision_pose_theta(robot->angle);

    const Planning::Path& robot_path = robot->path();

    RJ::Seconds time_elapsed = RJ::now() - robot_path.startTime();

    // Use this dt to calculate a finite-difference acceleration approximation.
    double dt = 1e-2;

    boost::optional<Planning::RobotInstant>
        maybe_instant_now = robot_path.evaluate(time_elapsed),
        maybe_instant_next = robot_path.evaluate(time_elapsed + RJ::Seconds(1e-2));

    // Unpack the current instant
    if (maybe_instant_now) {
        Planning::RobotInstant instant_now = *maybe_instant_now;
        robot->control->set_goal_pose_x(instant_now.motion.pos.x());
        robot->control->set_goal_pose_y(instant_now.motion.pos.y());

        robot->control->set_goal_velocity_x(instant_now.motion.vel.y());
        robot->control->set_goal_velocity_y(-instant_now.motion.vel.x());

        // Default angular values, in case there's no angular command
        robot->control->set_goal_pose_theta(robot->angle);
        robot->control->set_goal_velocity_theta(robot->angleVel);

        // Default acceleration values, in case there's no next command
        robot->control->set_goal_acceleration_x(0);
        robot->control->set_goal_acceleration_y(0);
        robot->control->set_goal_acceleration_theta(0);

        // Only set an angular command different from default if one exists.
        if (instant_now.angle) {
            if (instant_now.angle->angle) {
                robot->control->set_goal_pose_theta(*(instant_now.angle->angle));
            }
            if (instant_now.angle->angleVel) {
                robot->control->set_goal_velocity_theta(*(instant_now.angle->angleVel));
            }
        }

        // Calculate linear acceleration
        if (maybe_instant_next) {
            Planning::RobotInstant instant_next = *maybe_instant_next;
            double delta_vx = instant_next.motion.vel.x() - instant_now.motion.vel.x();
            double delta_vy = instant_next.motion.vel.y() - instant_now.motion.vel.y();
            robot->control->set_goal_acceleration_x(delta_vx / dt);
            robot->control->set_goal_acceleration_y(delta_vy / dt);

            // If we have angular velocity at this time _and_ the next, set angular acceleration.
            if (instant_now.angle && instant_now.angle->angleVel &&
                    instant_next.angle && instant_next.angle->angleVel) {
                double next_angle_vel = *(instant_next.angle->angleVel),
                       now_angle_vel = *(instant_next.angle->angleVel);
                double delta_vh = next_angle_vel - now_angle_vel;
                robot->control->set_goal_acceleration_theta(delta_vh / dt);
            }
        }

        robot->control->set_motion_mode(Packet::Control::POSITION_CONTROL);
    } else {
        // Stop the robot if we don't have a path.
        stop();
        robot->state()->drawCircle(robot->pos, .15, Qt::red,
                                    "Planning");
    }
}
