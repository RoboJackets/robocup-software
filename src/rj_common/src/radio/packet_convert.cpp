#include "rj_common/radio/packet_convert.hpp"

using soccer::robot::PARAM_max_chip_speed;
using soccer::robot::PARAM_max_kick_speed;
using soccer::robot::PARAM_min_chip_speed;
using soccer::robot::PARAM_min_kick_speed;
using soccer::robot::PARAM_max_dribbler_speed;
using soccer::robot::PARAM_chip_angle;

static uint8_t kicker_speed_to_strength(double kick_speed) {
    return static_cast<uint8_t>(kick_speed);
}

static uint8_t chipper_speed_to_strength(double kick_speed) {
    return static_cast<uint8_t>(std::min(1.0, (kick_speed - PARAM_min_chip_speed) /
                                                  (PARAM_max_chip_speed - PARAM_min_chip_speed)) *
                                kMaxKick);
}

namespace ConvertRx {

void rtp_to_status(const RadioMessage::RobotStatusMessage& rtp_message, RobotStatus* status) {
    if (status == nullptr) {
        return;
    }

    status->shell_id = rtp_message.robot_id;

    status->timestamp = RJ::now();
    status->version = RobotStatus::HardwareVersion::kFleet2018;
    status->twist_estimate = std::nullopt;
    status->pose_estimate = std::nullopt;
    status->battery_voltage = static_cast<float>(rtp_message.battery_voltage) *
                              RadioMessage::RobotStatusMessage::BATTERY_SCALE_FACTOR;
    status->kicker_voltage = 0;
    status->has_ball = rtp_message.ball_sense_status;
    status->kicker = rtp_message.kick_healthy
                         ? (rtp_message.kick_status ? RobotStatus::KickerState::kCharged
                                                    : RobotStatus::KickerState::kCharging)
                         : RobotStatus::KickerState::kFailed;
    for (int i = 0; i < 5; i++) {
        status->motors_healthy[i] = (rtp_message.motor_errors & (1u << i)) == 0;
    }
    status->fpga_healthy = rtp_message.fpga_status == 0u;
}

void sim_to_status(const RobotFeedback& sim, RobotStatus* status) {
    if (status == nullptr) {
        return;
    }

    status->shell_id = sim.id();
    status->timestamp = RJ::now();
    status->version = RobotStatus::HardwareVersion::kSimulated;
    status->twist_estimate = std::nullopt;
    status->pose_estimate = std::nullopt;

    // Field view is dumb and ignores hardware version, so give it a fake
    // battery voltage for sim
    status->battery_voltage = 20.0;

    status->kicker_voltage = 0;
    status->has_ball = sim.dribbler_ball_contact();

    status->kicker = RobotStatus::KickerState::kCharged;
    for (int i = 0; i < 5; i++) {
        status->motors_healthy[i] = true;
    }
    status->fpga_healthy = true;
}

void status_to_ros(const RobotStatus& status, rj_msgs::msg::RobotStatus* msg) {
    rj_convert::convert_to_ros(status.timestamp, &(msg->timestamp));
    msg->robot_id = status.shell_id;
    msg->battery_voltage = status.battery_voltage;
    msg->motor_errors = status.motors_healthy;
    msg->has_ball_sense = status.has_ball;
    msg->kicker_charged = status.kicker == RobotStatus::KickerState::kCharged;
    msg->kicker_healthy = status.kicker != RobotStatus::KickerState::kFailed;
    msg->fpga_error = !status.fpga_healthy;

    // TODO(Kyle): Handle encoders.
}

void ros_to_status(const rj_msgs::msg::RobotStatus& msg, RobotStatus* status) {
    rj_convert::convert_from_ros(msg.timestamp, &(status->timestamp));
    status->shell_id = msg.robot_id;
    status->battery_voltage = msg.battery_voltage;
    status->motors_healthy = msg.motor_errors;
    status->has_ball = msg.has_ball_sense;
    if (!msg.kicker_healthy) {
        status->kicker = RobotStatus::KickerState::kFailed;
    } else if (msg.kicker_charged) {
        status->kicker = RobotStatus::KickerState::kCharged;
    } else {
        status->kicker = RobotStatus::KickerState::kCharging;
    }
    status->fpga_healthy = !msg.fpga_error;

    // TODO(Kyle): Handle encoders.
}

}  // namespace ConvertRx

namespace ConvertTx {

// NOLINT(cppcoreguidelines-pro-type-union-access)
void to_rtp(const RobotIntent& intent, const MotionSetpoint& setpoint, int shell,
            RadioMessage::ControlMessage* rtp_message, bool blue_team) {
    rtp_message->team = blue_team;
    rtp_message->robot_id = shell;

    rtp_message->body_x = static_cast<int16_t>(setpoint.xvelocity *
                                               RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp_message->body_y = static_cast<int16_t>(setpoint.yvelocity *
                                               RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp_message->body_w = static_cast<int16_t>(setpoint.avelocity *
                                               RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp_message->dribbler_speed = std::clamp<uint16_t>(
        static_cast<uint16_t>(
            (intent.dribbler_mode == RobotIntent::DribblerMode::ON ? kMaxDribble : 0.0)),
        0, 255);

    if (intent.shoot_mode == RobotIntent::ShootMode::CHIP) {
        rtp_message->shoot_mode = 1;
        rtp_message->kick_strength = chipper_speed_to_strength(intent.kick_speed);
    } else {
        rtp_message->shoot_mode = 0;
        rtp_message->kick_strength = kicker_speed_to_strength(intent.kick_speed);
    }

    switch (intent.trigger_mode) {
        case RobotIntent::TriggerMode::STAND_DOWN:
            rtp_message->trigger_mode = 0;
            break;
        case RobotIntent::TriggerMode::IMMEDIATE:
            rtp_message->trigger_mode = 1;
            break;
        case RobotIntent::TriggerMode::ON_BREAK_BEAM:
            rtp_message->trigger_mode = 2;
            break;
        default:
            break;
    }
}

void to_sim(const RobotIntent& intent, const MotionSetpoint& setpoint, int shell,
            RobotCommand* sim) {
    if (sim == nullptr) {
        return;
    }

    sim->set_id(shell);

    if (intent.trigger_mode == RobotIntent::TriggerMode::STAND_DOWN) {
        sim->set_kick_speed(0);
        sim->set_kick_angle(0);
    } else {
        if (intent.shoot_mode == RobotIntent::ShootMode::KICK) {
            // Flat kick
            sim->set_kick_speed(intent.kick_speed);
            sim->set_kick_angle(0);
        } else {
            // Chip kick
            sim->set_kick_speed(intent.kick_speed);
            // Chip angle is already in degrees
            sim->set_kick_angle(static_cast<float>(PARAM_chip_angle));
        }
    }

    auto* command = sim->mutable_move_command()->mutable_local_velocity();
    command->set_forward(static_cast<float>(setpoint.yvelocity));
    command->set_left(-static_cast<float>(setpoint.xvelocity));
    command->set_angular(static_cast<float>(setpoint.avelocity));

    sim->set_dribbler_speed(
        static_cast<float>(PARAM_max_dribbler_speed *
                           (intent.dribbler_mode == RobotIntent::DribblerMode::ON ? 255.0 : 0.0)));
}
void ros_to_rtp(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell,
                RadioMessage::ControlMessage* rtp, strategy::Positions role, bool blue_team) {
    rtp->team = blue_team;
    rtp->robot_id = shell;
    rtp->body_x = static_cast<int16_t>(motion.velocity_x_mps *
                                       RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp->body_y = static_cast<int16_t>(motion.velocity_y_mps *
                                       RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp->body_w = static_cast<int16_t>(motion.velocity_z_radps *
                                       RadioMessage::ControlMessage::VELOCITY_SCALE_FACTOR);
    rtp->dribbler_speed = std::max(0.0f, std::min(manipulator.dribbler_speed, 125.0f));
    if (manipulator.shoot_mode == rj_msgs::msg::ManipulatorSetpoint::SHOOT_MODE_KICK) {
        rtp->kick_strength = kicker_speed_to_strength(manipulator.kick_speed);
    } else {
        rtp->kick_strength = chipper_speed_to_strength(manipulator.kick_speed);
    }
    rtp->shoot_mode = manipulator.shoot_mode;
    rtp->trigger_mode = manipulator.trigger_mode;
    rtp->role = role;
}

void ros_to_sim(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                  const rj_msgs::msg::MotionSetpoint& motion, int shell,
                  RobotCommand* sim) {
    if (sim == nullptr) {
        return;
    }

    sim->set_id(shell);

    if (manipulator.trigger_mode == static_cast<uint8_t>(RobotIntent::TriggerMode::STAND_DOWN)) {
        sim->set_kick_speed(0);
        sim->set_kick_angle(0);
    } else {
        if (manipulator.shoot_mode == static_cast<uint8_t>(RobotIntent::ShootMode::KICK)) {
            // Flat kick
            float speed = manipulator.kick_speed;
            sim->set_kick_speed(speed);
            sim->set_kick_angle(0);
        } else {
            // Chip kick
            float speed = manipulator.kick_speed;
            sim->set_kick_speed(speed);
            sim->set_kick_angle(static_cast<float>(PARAM_chip_angle));
        }
    }

    auto* command = sim->mutable_move_command()->mutable_local_velocity();
    command->set_forward(static_cast<float>(motion.velocity_y_mps));
    command->set_left(-static_cast<float>(motion.velocity_x_mps));
    command->set_angular(static_cast<float>(motion.velocity_z_radps));

    sim->set_dribbler_speed(static_cast<float>(PARAM_max_dribbler_speed * manipulator.dribbler_speed));
}

}  // namespace ConvertTx
