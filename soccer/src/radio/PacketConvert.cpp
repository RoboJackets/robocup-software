#include "PacketConvert.hpp"

#include <status.h>

#include <Geometry2d/Util.hpp>
#include <time.hpp>

#include "RobotIntent.hpp"
#include "RobotStatus.hpp"
#include "motion/MotionSetpoint.hpp"

namespace ConvertRx {

void rtp_to_status(const rtp::RobotStatusMessage& rtp_message,
                   RobotStatus* status) {
    if (status == nullptr) {
        return;
    }

    status->shell_id = rtp_message.uid;

    status->timestamp = RJ::now();
    status->version = RobotStatus::HardwareVersion::kFleet2018;
    status->twist_estimate = std::nullopt;
    status->pose_estimate = std::nullopt;
    status->battery_voltage = static_cast<float>(rtp_message.battVoltage) *
                              rtp::RobotStatusMessage::BATTERY_SCALE_FACTOR;
    status->kicker_voltage = 0;
    status->has_ball = rtp_message.ballSenseStatus;
    status->kicker =
        rtp_message.kickHealthy
            ? (rtp_message.kickStatus ? RobotStatus::KickerState::kCharged
                                      : RobotStatus::KickerState::kCharging)
            : RobotStatus::KickerState::kFailed;
    for (int i = 0; i < 5; i++) {
        status->motors_healthy[i] = (rtp_message.motorErrors & (1u << i)) == 0;
    }
    status->fpga_healthy = rtp_message.fpgaStatus == 0u;
}

void grsim_to_status(const Robot_Status& grsim, RobotStatus* status) {
    if (status == nullptr) {
        return;
    }

    status->shell_id = grsim.robot_id();
    status->timestamp = RJ::now();
    status->version = RobotStatus::HardwareVersion::kSimulated;
    status->twist_estimate = std::nullopt;
    status->pose_estimate = std::nullopt;

    // Field view is dumb and ignores hardware version, so give it a fake
    // battery voltage for grSim
    status->battery_voltage = 20.0;

    status->kicker_voltage = 0;
    status->has_ball = grsim.infrared();

    bool kicked = grsim.chip_kick() || grsim.flat_kick();
    status->kicker = kicked ? RobotStatus::KickerState::kCharging
                            : RobotStatus::KickerState::kCharged;
    for (int i = 0; i < 5; i++) {
        status->motors_healthy[i] = true;
    }
    status->fpga_healthy = true;
}

void status_to_proto(const RobotStatus& status, Packet::RadioRx* proto) {
    using namespace std::chrono;

    proto->set_timestamp(
        duration_cast<microseconds>(status.timestamp.time_since_epoch())
            .count());
    proto->set_robot_id(status.shell_id);
    proto->set_battery(static_cast<float>(status.battery_voltage));

    proto->set_ball_sense_status(status.has_ball
                                     ? Packet::BallSenseStatus::HasBall
                                     : Packet::BallSenseStatus::NoBall);

    for (int i = 0; i < 5; i++) {
        proto->add_motor_status(status.motors_healthy[i]
                                    ? Packet::MotorStatus::Good
                                    : Packet::MotorStatus::Encoder_Failure);
    }

    // No encoders or RSSI

    switch (status.kicker) {
        case RobotStatus::KickerState::kFailed:
            proto->set_kicker_status(Kicker_I2C_OK);
            break;
        case RobotStatus::KickerState::kCharging:
            proto->set_kicker_status(Kicker_Enabled | Kicker_I2C_OK);
            break;
        case RobotStatus::KickerState::kCharged:
            proto->set_kicker_status(Kicker_Charged | Kicker_Enabled |
                                     Kicker_I2C_OK);
            break;
    }

    proto->set_kicker_voltage(status.kicker_voltage);
    proto->set_fpga_status(status.fpga_healthy ? Packet::FpgaStatus::FpgaGood
                                               : Packet::FpgaStatus::FpgaError);

    switch (status.version) {
        case RobotStatus::HardwareVersion::kUnknown:
            proto->set_hardware_version(Packet::HardwareVersion::Unknown);
            break;
        case RobotStatus::HardwareVersion::kFleet2018:
            proto->set_hardware_version(Packet::HardwareVersion::RJ2018);
            break;
        case RobotStatus::HardwareVersion::kSimulated:
            proto->set_hardware_version(Packet::HardwareVersion::Simulation);
            break;
    }
}

}  // namespace ConvertRx

namespace ConvertTx {

// NOLINT(cppcoreguidelines-pro-type-union-access)
void to_rtp(const RobotIntent& intent, const MotionSetpoint& setpoint,
            int shell, rtp::RobotTxMessage* rtp_message) {
    rtp_message->uid = shell;
    rtp::ControlMessage controlMessage{};

    controlMessage.bodyX = static_cast<int16_t>(
        setpoint.xvelocity * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    controlMessage.bodyY = static_cast<int16_t>(
        setpoint.yvelocity * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    controlMessage.bodyW = static_cast<int16_t>(
        setpoint.avelocity * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
    controlMessage.dribbler =
        std::clamp(static_cast<uint16_t>(intent.dvelocity) * 2, 0, 255);

    controlMessage.shootMode =
        intent.shoot_mode == RobotIntent::ShootMode::CHIP;
    controlMessage.kickStrength = intent.kcstrength;

    switch (intent.trigger_mode) {
        case RobotIntent::TriggerMode::STAND_DOWN:
            controlMessage.triggerMode = 0;
            break;
        case RobotIntent::TriggerMode::IMMEDIATE:
            controlMessage.triggerMode = 1;
            break;
        case RobotIntent::TriggerMode::ON_BREAK_BEAM:
            controlMessage.triggerMode = 2;
            break;
    }

    switch (intent.song) {
        case RobotIntent::Song::STOP:
            controlMessage.song = 0;
            break;
        case RobotIntent::Song::CONTINUE:
            controlMessage.song = 1;
            break;
        case RobotIntent::Song::FIGHT_SONG:
            controlMessage.song = 2;
            break;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access)
    rtp_message->message.controlMessage = controlMessage;

    rtp_message->messageType = rtp::RobotTxMessage::ControlMessageType;
}

void to_proto(const RobotIntent& intent, const MotionSetpoint& setpoint,
              int shell, Packet::Robot* proto) {
    if (proto == nullptr) {
        return;
    }

    proto->set_uid(shell);

    Packet::Control* control = proto->mutable_control();

    control->set_xvelocity(static_cast<float>(setpoint.xvelocity));
    control->set_yvelocity(static_cast<float>(setpoint.yvelocity));
    control->set_avelocity(static_cast<float>(setpoint.avelocity));
    control->set_dvelocity(intent.dvelocity);
    control->set_kcstrength(intent.kcstrength);

    switch (intent.shoot_mode) {
        case RobotIntent::ShootMode::KICK:
            control->set_shootmode(Packet::Control_ShootMode_KICK);
            break;
        case RobotIntent::ShootMode::CHIP:
            control->set_shootmode(Packet::Control_ShootMode_CHIP);
            break;
    }

    switch (intent.trigger_mode) {
        case RobotIntent::TriggerMode::STAND_DOWN:
            control->set_triggermode(Packet::Control_TriggerMode_STAND_DOWN);
            break;
        case RobotIntent::TriggerMode::IMMEDIATE:
            control->set_triggermode(Packet::Control_TriggerMode_IMMEDIATE);
            break;
        case RobotIntent::TriggerMode::ON_BREAK_BEAM:
            control->set_triggermode(Packet::Control_TriggerMode_ON_BREAK_BEAM);
            break;
    }

    switch (intent.song) {
        case RobotIntent::Song::STOP:
            control->set_song(Packet::Control_Song_STOP);
            break;
        case RobotIntent::Song::CONTINUE:
            control->set_song(Packet::Control_Song_CONTINUE);
            break;
        case RobotIntent::Song::FIGHT_SONG:
            control->set_song(Packet::Control_Song_FIGHT_SONG);
            break;
    }
}

void to_grsim(const RobotIntent& intent, const MotionSetpoint& setpoint,
              int shell, grSim_Robot_Command* grsim) {
    if (grsim == nullptr) {
        return;
    }

    grsim->set_id(shell);

    if (intent.trigger_mode == RobotIntent::TriggerMode::STAND_DOWN) {
        grsim->set_kickspeedx(0);
        grsim->set_kickspeedz(0);
    } else {
        if (intent.shoot_mode == RobotIntent::ShootMode::KICK) {
            // Flat kick
            constexpr double kMaxKickSpeed = 7.0;
            constexpr double kMinKickSpeed = 2.1;
            constexpr double kStrengthToSpeed =
                (kMaxKickSpeed - kMinKickSpeed) / 255;
            double speed = kStrengthToSpeed * intent.kcstrength + kMinKickSpeed;
            grsim->set_kickspeedx(static_cast<float>(speed));
            grsim->set_kickspeedz(0);
        } else {
            // Chip kick
            constexpr double kMaxChipSpeed = 4.0;
            constexpr double kMinChipSpeed = 1.0;
            constexpr double kStrengthToSpeed =
                (kMaxChipSpeed - kMinChipSpeed) / 255;
            constexpr double kChipAngle = 40 * M_PI / 180;  // degrees

            double speed = kStrengthToSpeed * intent.kcstrength + kMinChipSpeed;
            grsim->set_kickspeedx(
                static_cast<float>(std::cos(kChipAngle) * speed));
            grsim->set_kickspeedz(
                static_cast<float>(std::sin(kChipAngle) * speed));
        }
    }

    grsim->set_veltangent(static_cast<float>(setpoint.yvelocity));
    grsim->set_velnormal(-static_cast<float>(setpoint.xvelocity));
    grsim->set_velangular(static_cast<float>(setpoint.avelocity));

    grsim->set_spinner(intent.dvelocity > 0);
    grsim->set_wheelsspeed(false);
}

}  // namespace ConvertTx

void fill_header(rtp::Header* header) {
    header->port = rtp::PortType::CONTROL;
    header->address = rtp::BROADCAST_ADDRESS;
    header->type = rtp::MessageType::CONTROL;
}
