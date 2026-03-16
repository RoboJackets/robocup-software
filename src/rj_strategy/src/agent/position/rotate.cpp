#include "rotate.hpp"
#include <cmath>

// Add your team's specific headers for geometry and ROS messages here:
// #include "rj_geometry/pose.hpp"
// #include "rj_geometry/point.hpp"
// #include "rj_msgs/msg/path_target_motion_command.hpp"

namespace strategy {

// --- Constructors ---

Rotate::Rotate(int r_id)
    : Position(r_id, "Rotate") {
}

Rotate::Rotate(int r_id, bool clockwise, bool sweep_mode)
    : Position(r_id, "Rotate"),
      clockwise_{clockwise},
      sweep_mode_{sweep_mode} {
}

Rotate::Rotate(Position&& other)
    : Position(std::move(other)) {
}

std::string Rotate::get_current_state() {
    switch (state_) {
        case RotateState::DEG_0:   return "Rotate: 0 Degrees";
        case RotateState::DEG_90:  return "Rotate: 90 Degrees";
        case RotateState::DEG_180: return "Rotate: 180 Degrees";
        case RotateState::DEG_270: return "Rotate: 270 Degrees";
        default:                   return "Rotate: Unknown State";
    }
}

std::optional<RobotIntent> Rotate::derived_get_task(RobotIntent intent) {
    // 1. Map the current state to a target angle in radians
    double target_angle = 0.0;
    switch (state_) {
        case RotateState::DEG_0:   target_angle = 0.0; break;
        case RotateState::DEG_90:  target_angle = M_PI / 2.0; break;
        case RotateState::DEG_180: target_angle = M_PI; break;
        case RotateState::DEG_270: target_angle = 3.0 * M_PI / 2.0; break;
    }

    // 2. Execute sweep logic if enabled and world state is available
    if (sweep_mode_ && last_world_state_ != nullptr) {

        // Get the robot's current pose
        // Note: Update this syntax if your WorldState struct differs
        double current_angle = last_world_state_->our_robots.at(robot_id_).pose.heading();

        // Calculate the difference between current and target angle
        // Replace this block with your codebase's angle normalization utility if you have one
        double angle_diff = std::abs(current_angle - target_angle);
        if (angle_diff > M_PI) {
            angle_diff = 2.0 * M_PI - angle_diff;
        }

        // Tolerance for reaching the target angle (~5.7 degrees)
        bool reached_target = (angle_diff < 0.1);

        if (reached_target) {
            // State Machine Transitions
            if (sweeping_up_) {
                if (state_ == RotateState::DEG_0) {
                    state_ = RotateState::DEG_90;
                } else if (state_ == RotateState::DEG_90) {
                    state_ = RotateState::DEG_180;
                } else if (state_ == RotateState::DEG_180) {
                    state_ = RotateState::DEG_270;
                } else if (state_ == RotateState::DEG_270) {
                    sweeping_up_ = false; // Reverse direction
                    state_ = RotateState::DEG_180;
                }
            } else { // sweeping down
                if (state_ == RotateState::DEG_270) {
                    state_ = RotateState::DEG_180;
                } else if (state_ == RotateState::DEG_180) {
                    state_ = RotateState::DEG_90;
                } else if (state_ == RotateState::DEG_90) {
                    state_ = RotateState::DEG_0;
                } else if (state_ == RotateState::DEG_0) {
                    sweeping_up_ = true; // Reverse direction
                    state_ = RotateState::DEG_90;
                }
            }
        }
    }

    // 3. Build the Motion Command
    if (last_world_state_ != nullptr) {
        rj_msgs::msg::PathTargetMotionCommand ptmc;

        // Keep the robot in its current physical location (rotate in place)
        auto current_position = last_world_state_->our_robots.at(robot_id_).pose.position();

        // You may need an rj_geometry conversion function here:
        // ptmc.target.position = rj_geometry::to_msg(current_position);

        rj_msgs::msg::HeadingTarget heading_target;
        heading_target.target_heading = target_angle;
        ptmc.override_heading = {heading_target};

        // Assign the command to the intent
        intent.motion_command.path_target_command = {ptmc};
    }

    return intent;
}

} // namespace strategy
