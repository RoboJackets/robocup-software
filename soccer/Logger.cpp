#include "Logger.hpp"

#include "Context.hpp"
#include "radio/PacketConvert.hpp"

using namespace Packet;

void Logger::start() { _context->logs.start_time = RJ::now(); }

void Logger::run() {
    // Add everything to the log frame.
    auto log_frame = std::make_shared<LogFrame>();

    // Debug drawing
    _context->debug_drawer.fillLogFrame(log_frame.get());

    for (const SSL_WrapperPacket& packet : _context->raw_vision_packets) {
        log_frame->add_raw_vision()->CopyFrom(packet);
    }
    _context->raw_vision_packets.clear();

    for (const auto& packet : _context->referee_packets) {
        SSL_Referee* referee = log_frame->add_raw_refbox();
        referee->CopyFrom(packet);
    }

    log_frame->set_blue_team(_context->game_state.blueTeam);
    log_frame->set_command_time(RJ::timestamp());

    // Our robots
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& state = _context->world_state.our_robots.at(shell);
        const auto& status = _context->robot_status.at(shell);
        const auto& intent = _context->robot_intents.at(shell);
        const auto& setpoint = _context->motion_setpoints.at(shell);

        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = log_frame->add_self();

        robot->mutable_pos()->set_x(state.pose.position().x());
        robot->mutable_pos()->set_y(state.pose.position().y());
        robot->set_angle(state.pose.heading());
        robot->mutable_world_vel()->set_x(state.velocity.linear().x());
        robot->mutable_world_vel()->set_y(state.velocity.linear().y());

        // TODO body vel?

        robot->set_shell(shell);

        robot->set_ball_sense_status(status.has_ball ? BallSenseStatus::HasBall
                                                     : BallSenseStatus::NoBall);

        // TODO debug text

        robot->mutable_cmd_vel()->set_x(setpoint.xvelocity);
        robot->mutable_cmd_vel()->set_y(setpoint.yvelocity);
        robot->set_cmd_w(setpoint.avelocity);

        robot->set_charged(status.kicker == RobotStatus::KickerState::kCharged);
        robot->set_kicker_voltage(status.kicker_voltage);

        for (int i = 0; i < 5; i++) {
            robot->add_motor_status(status.motors_healthy[i]
                                        ? MotorStatus::Good
                                        : MotorStatus::Encoder_Failure);
        }

        robot->set_kicker_works(status.kicker !=
                                RobotStatus::KickerState::kFailed);
        robot->set_battery_voltage(status.battery_voltage);
    }

    // Radio RX/TX for each robot
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& status = _context->robot_status.at(shell);
        if (RJ::now() - status.timestamp > RJ::Seconds(0.5)) {
            continue;
        }
        const auto& intent = _context->robot_intents.at(shell);
        const auto& setpoint = _context->motion_setpoints.at(shell);

        RadioRx* rx = log_frame->add_radio_rx();
        ConvertRx::status_to_proto(status, rx);

        Packet::Robot* tx = log_frame->mutable_radio_tx()->add_robots();
        ConvertTx::to_proto(intent, setpoint, shell, tx);
    }

    // Opponent robots
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& state = _context->world_state.their_robots.at(shell);
        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = log_frame->add_opp();

        robot->mutable_pos()->set_x(state.pose.position().x());
        robot->mutable_pos()->set_y(state.pose.position().y());
        robot->set_angle(state.pose.heading());
        robot->mutable_world_vel()->set_x(state.velocity.linear().x());
        robot->mutable_world_vel()->set_y(state.velocity.linear().y());

        robot->set_shell(shell);

        // TODO body vel?
    }

    // Ball
    if (_context->state.ball.valid) {
        log_frame->mutable_ball()->mutable_pos()->set_x(
            _context->state.ball.pos.x());
        log_frame->mutable_ball()->mutable_pos()->set_y(
            _context->state.ball.pos.y());
        log_frame->mutable_ball()->mutable_vel()->set_x(
            _context->state.ball.vel.x());
        log_frame->mutable_ball()->mutable_vel()->set_y(
            _context->state.ball.vel.y());
    }

    log_frame->set_manual_id(_context->game_settings.joystick_config.manualID);
    log_frame->set_defend_plus_x(_context->game_settings.defendPlusX);
    log_frame->set_use_our_half(_context->game_settings.use_our_half);
    log_frame->set_use_opponent_half(_context->game_settings.use_their_half);

    // Behavior tree
    log_frame->set_behavior_tree(_context->behavior_tree);

    // Team names
    if (_context->game_state.blueTeam) {
        log_frame->set_team_name_yellow(_context->game_state.TheirInfo.name);
        log_frame->set_team_name_blue(_context->game_state.OurInfo.name);
    } else {
        log_frame->set_team_name_yellow(_context->game_state.OurInfo.name);
        log_frame->set_team_name_blue(_context->game_state.TheirInfo.name);
    }

    log_frame->set_timestamp(RJ::timestamp());

    if (_log_file.has_value() && _log_file->is_open()) {
        _context->logs.size_bytes += log_frame->ByteSizeLong();
        log_frame->SerializeToOstream(&_log_file.value());
    }

    while (_context->logs.frames.size() + 1 >= kMaxLogFrames) {
        _context->logs.frames.pop_front();
    }

    _context->logs.frames.emplace_back(std::move(log_frame));
}

void Logger::stop() {}

void Logger::read(const std::string& filename) {
    _log_file = std::fstream(filename);
    _context->logs.filename = filename;
    _state = Logs::State::kReading;

    // Populate the entire logs struct.
    auto frame = std::make_shared<LogFrame>();
    while (frame->ParseFromIstream(&_log_file.value())) {
        _context->logs.size_bytes += frame->ByteSizeLong();
        _context->logs.frames.emplace_back(std::move(frame));
        frame = std::make_shared<LogFrame>();
    }
}

void Logger::write(const std::string& filename) {
    _log_file = std::fstream(filename);
    _context->logs.filename = filename;
    _state = Logs::State::kWriting;
}

void Logger::close() {
    _log_file = std::nullopt;
    _context->logs.filename = std::nullopt;
    _state = Logs::State::kNoFile;
}
