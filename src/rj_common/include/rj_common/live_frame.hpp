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

enum BallSenseStatus { NoBall, HasBall, Dazzled, Failed };
enum MotorStatus { Good, Fault };

struct LiveRobot {
    int shell() const { return shell_id; }
    const rj_geometry::Point& pos() const { return position; }
    float angle() const { return static_cast<float>(heading); }
    bool has_ball_sense_status() const { return true; }
    BallSenseStatus ball_sense_status() const {
        return has_ball ? BallSenseStatus::HasBall : BallSenseStatus::NoBall;
    }
    bool has_kicker_works() const { return false; }
    bool kicker_works() const { return kicker_ok; }
    const std::vector<MotorStatus>& motor_status() const { return motors; }
    MotorStatus motor_status(int index) const { return motors.at(index); }
    bool has_battery_voltage() const { return true; }
    float battery_voltage() const { return battery; }
    const std::vector<DebugText>& text() const { return texts; }

    int shell_id = -1;
    rj_geometry::Point position;
    double heading = 0;
    bool has_ball = false;
    bool kicker_ok = true;
    float battery = 0;
    std::vector<MotorStatus> motors;
    std::vector<DebugText> texts;
};

struct LiveBall {
    const rj_geometry::Point& pos() const { return position; }
    const rj_geometry::Point& vel() const { return velocity; }
    rj_geometry::Point position;
    rj_geometry::Point velocity;
};

struct LiveFrame {
    using Robot = LiveRobot;

    bool defend_plus_x() const { return defend_plus; }
    bool use_our_half() const { return use_our; }
    bool use_opponent_half() const { return use_opponent; }
    bool blue_team() const { return blue; }
    int manual_id() const { return manual; }
    const std::string& team_name_blue() const { return blue_name; }
    const std::string& team_name_yellow() const { return yellow_name; }
    bool has_ball() const { return ball_state.has_value(); }
    const LiveBall& ball() const { return *ball_state; }
    const std::vector<DebugPath>& debug_paths() const { return debug_draw_frame.paths; }
    const std::vector<DebugRobotPath>& debug_robot_paths() const {
        return debug_draw_frame.robot_paths;
    }
    const std::vector<DebugCircle>& debug_circles() const { return debug_draw_frame.circles; }
    const std::vector<DebugArc>& debug_arcs() const { return debug_draw_frame.arcs; }
    const std::vector<DebugPath>& debug_polygons() const { return debug_draw_frame.polygons; }
    const std::vector<DebugText>& debug_texts() const { return debug_draw_frame.texts; }
    int debug_layers_size() const {
        return static_cast<int>(debug_draw_frame.debug_layers_.size());
    }
    const std::string& debug_layers(int i) const { return debug_draw_frame.debug_layers_.at(i); }

    const std::vector<LiveRobot>& self() const { return self_; }
    const std::vector<LiveRobot>& opp() const { return opp_; }
    const LiveRobot& self(int index) const { return self_.at(index); }
    const LiveRobot& opp(int index) const { return opp_.at(index); }
    int self_size() const { return static_cast<int>(self_.size()); }
    int opp_size() const { return static_cast<int>(opp_.size()); }

    std::vector<LiveRobot> self_;
    std::vector<LiveRobot> opp_;
    std::vector<RobotStatus> radio_rx_;
    std::optional<LiveBall> ball_state;
    bool blue = true;
    bool defend_plus = false;
    bool use_our = true;
    bool use_opponent = true;
    int manual = -1;
    std::string blue_name;
    std::string yellow_name;
    DebugDrawFrame debug_draw_frame;

    static void fill_robot(LiveRobot* out, int shell_id, RobotState const& state,
                           RobotStatus const* status) {
        out->shell_id = shell_id;

        out->position = state.pose.position();
        out->heading = state.pose.heading();

        if (status != nullptr) {
            out->has_ball = status->has_ball;
            out->motors.resize(5);
            for (int i = 0; i < 5; i++) {
                out->motors.at(i) =
                    (status->motors_healthy[i] ? MotorStatus::Good : MotorStatus::Fault);
            }
            out->kicker_ok = status->kicker != RobotStatus::KickerState::kFailed;
            out->battery = static_cast<float>(status->battery_voltage);
        }
    }
};

}  // namespace rj_common
