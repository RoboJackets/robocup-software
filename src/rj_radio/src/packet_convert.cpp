#include "rj_radio/packet_convert.hpp"

static uint8_t kicker_speed_to_strength(double kick_speed) {
    return static_cast<uint8_t>(kick_speed);
}

static uint8_t chipper_speed_to_strength(double kick_speed, const RobotHardwareParams& hardware) {
    return static_cast<uint8_t>(
        std::min(1.0, (kick_speed - hardware.min_chip_speed) /
                          (hardware.max_chip_speed - hardware.min_chip_speed)) *
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

}  // namespace ConvertRx

namespace ConvertTx {

void ros_to_rtp(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell,
                RadioMessage::ControlMessage* rtp, bool blue_team,
                const RobotHardwareParams& hardware) {
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
        rtp->kick_strength = chipper_speed_to_strength(manipulator.kick_speed, hardware);
    }
    rtp->shoot_mode = manipulator.shoot_mode;
    rtp->trigger_mode = manipulator.trigger_mode;
}

void ros_to_sim(const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                const rj_msgs::msg::MotionSetpoint& motion, int shell, RobotCommand* sim,
                const RobotHardwareParams& hardware) {
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
            sim->set_kick_angle(static_cast<float>(hardware.chip_angle));
        }
    }

    auto* command = sim->mutable_move_command()->mutable_local_velocity();
    command->set_forward(static_cast<float>(motion.velocity_y_mps));
    command->set_left(-static_cast<float>(motion.velocity_x_mps));
    command->set_angular(static_cast<float>(motion.velocity_z_radps));

    sim->set_dribbler_speed(
        static_cast<float>(hardware.max_dribbler_speed * manipulator.dribbler_speed));
}

}  // namespace ConvertTx
