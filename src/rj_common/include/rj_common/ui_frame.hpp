#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rj_common/control/motion_setpoint.hpp"
#include "rj_common/debug_drawings.hpp"
#include "rj_common/radio/robot_status.hpp"
#include "rj_common/robot_intent.hpp"
#include "rj_common/time.hpp"
#include "rj_common/world_state.hpp"

namespace rj_common {

enum BallSenseStatus { kNoBall = 0, kHasBall, kDazzled, kFailed };
enum MotorStatus { kGood = 0, kFault };

struct UIRobot {
    UIRobot(int shell_id, const RobotState& state,
            const std::optional<RobotStatus>& status) {
        this->shell_id = shell_id;

        position = state.pose.position();
        heading = state.pose.heading();

        if (status != std::nullopt) {
            has_ball = status->has_ball;
            motors.resize(5);
            for (int i = 0; i < 5; i++) {
                motors.at(i) = (status->motors_healthy[i] ? rj_common::MotorStatus::kGood
                                                        : rj_common::MotorStatus::kFault);
            }
            kicker_ok = status->kicker != RobotStatus::KickerState::kFailed;
            battery = static_cast<float>(status->battery_voltage);
        }
    }

    int shell_id = -1;
    rj_geometry::Point position;
    double heading = 0;
    bool has_ball = false;
    bool kicker_ok = true;
    float battery = 0;
    std::vector<MotorStatus> motors;
    std::vector<DebugText> texts;
};

struct UIBall {
    rj_geometry::Point position;
    rj_geometry::Point velocity;
};

struct UIFrame {
    std::vector<UIRobot> self;
    std::vector<UIRobot> opp;
    std::vector<RobotStatus> radio_rx;
    std::optional<UIBall> ball_state;
    bool blue = true;
    bool defend_plus_x = false;
    bool use_our_half = true;
    bool use_their_half = true;
    int manual_id = -1;
    std::string blue_name;
    std::string yellow_name;
    DebugDrawFrame debug_draw_frame;
};

}  // namespace rj_common
