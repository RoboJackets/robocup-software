#pragma once

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rj_common/context.hpp>

namespace rj_ui {

enum BallSenseStatus { NoBall, HasBall, Dazzled, Failed };
enum MotorStatus { Good, Fault };

struct DebugPath {
    int layer() const { return -1; }
    int color() const { return 0; }
    int points_size() const { return 0; }
    rj_geometry::Point points(int) const { return {}; }
};
struct DebugRobotPath {
    struct DebugRobotPathPoint {
        rj_geometry::Point pos() const { return {}; }
        rj_geometry::Point vel() const { return {}; }
    };
    int layer() const { return -1; }
    int points_size() const { return 0; }
    DebugRobotPathPoint points(int) const { return {}; }
};
struct DebugCircle {
    int layer() const { return -1; }
    int color() const { return 0; }
    rj_geometry::Point center() const { return {}; }
    float radius() const { return 0; }
};
struct DebugArc {
    int layer() const { return -1; }
    int color() const { return 0; }
    rj_geometry::Point center() const { return {}; }
    float radius() const { return 0; }
    float start() const { return 0; }
    float end() const { return 0; }
};
struct DebugText {
    int layer() const { return -1; }
    int color() const { return 0; }
    rj_geometry::Point pos() const { return {}; }
    std::string text() const { return {}; }
    bool center() const { return true; }
};

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
    static LiveFrame from_context(const Context& context) {
        LiveFrame frame;
        frame.blue = context.blue_team;
        frame.defend_plus = context.game_settings.defend_plus_x;
        frame.use_our = context.game_settings.use_our_half;
        frame.use_opponent = context.game_settings.use_their_half;
        frame.manual = context.game_settings.joystick_config.manual_id;
        frame.blue_name = context.blue_team ? context.our_info.name : context.their_info.name;
        frame.yellow_name = context.blue_team ? context.their_info.name : context.our_info.name;

        for (size_t shell = 0; shell < context.world_state.our_robots.size(); ++shell) {
            const auto& state = context.world_state.our_robots[shell];
            if (!state.visible) continue;
            LiveRobot robot;
            robot.shell_id = static_cast<int>(shell);
            robot.position = state.pose.position();
            robot.heading = state.pose.heading();
            const auto& status = context.robot_status[shell];
            robot.has_ball = status.has_ball;
            robot.kicker_ok = status.kicker != RobotStatus::KickerState::kFailed;
            robot.battery = static_cast<float>(status.battery_voltage);
            robot.motors.resize(status.motors_healthy.size(), MotorStatus::Good);
            frame.self_.push_back(std::move(robot));
        }
        for (size_t shell = 0; shell < context.world_state.their_robots.size(); ++shell) {
            const auto& state = context.world_state.their_robots[shell];
            if (!state.visible) continue;
            LiveRobot robot;
            robot.shell_id = static_cast<int>(shell);
            robot.position = state.pose.position();
            robot.heading = state.pose.heading();
            frame.opp_.push_back(std::move(robot));
        }
        if (context.world_state.ball.visible) {
            frame.ball_state =
                LiveBall{context.world_state.ball.position, context.world_state.ball.velocity};
        }
        return frame;
    }

    bool defend_plus_x() const { return defend_plus; }
    bool use_our_half() const { return use_our; }
    bool use_opponent_half() const { return use_opponent; }
    bool blue_team() const { return blue; }
    int manual_id() const { return manual; }
    const std::string& team_name_blue() const { return blue_name; }
    const std::string& team_name_yellow() const { return yellow_name; }
    bool has_ball() const { return ball_state.has_value(); }
    const LiveBall& ball() const { return *ball_state; }
    const std::vector<DebugPath>& debug_paths() const { return debug_paths_; }
    const std::vector<DebugRobotPath>& debug_robot_paths() const { return debug_robot_paths_; }
    const std::vector<DebugCircle>& debug_circles() const { return debug_circles_; }
    const std::vector<DebugArc>& debug_arcs() const { return debug_arcs_; }
    const std::vector<DebugPath>& debug_polygons() const { return debug_polygons_; }
    const std::vector<DebugText>& debug_texts() const { return debug_texts_; }

    const std::vector<LiveRobot>& self_robots() const { return self_; }
    const std::vector<LiveRobot>& opp_robots() const { return opp_; }
    const std::vector<LiveRobot>& self() const { return self_; }
    const std::vector<LiveRobot>& opp() const { return opp_; }
    const LiveRobot& self(int index) const { return self_.at(index); }
    const LiveRobot& opp(int index) const { return opp_.at(index); }
    int self_size() const { return static_cast<int>(self_.size()); }
    int opp_size() const { return static_cast<int>(opp_.size()); }

    std::vector<LiveRobot> self_;
    std::vector<LiveRobot> opp_;
    std::optional<LiveBall> ball_state;
    bool blue = true;
    bool defend_plus = false;
    bool use_our = true;
    bool use_opponent = true;
    int manual = -1;
    std::string blue_name;
    std::string yellow_name;
    std::vector<DebugPath> debug_paths_;
    std::vector<DebugRobotPath> debug_robot_paths_;
    std::vector<DebugCircle> debug_circles_;
    std::vector<DebugArc> debug_arcs_;
    std::vector<DebugPath> debug_polygons_;
    std::vector<DebugText> debug_texts_;
};

}  // namespace rj_ui
