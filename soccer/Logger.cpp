#include "Logger.hpp"

using namespace Packet;

void Logger::start() {
}

void Logger::run() {
    // Add everything to the log frame.
    LogFrame* log_frame = _context->logFrame;

    for (const auto& packet : _context->vision_packets) {
        SSL_WrapperPacket* vision = _context->logFrame->add_raw_vision();
        vision->CopyFrom(packet->wrapper);
    }

    for (const auto& packet : _context->referee_packets) {
        SSL_Referee* referee = _context->logFrame->add_raw_refbox();
        referee->CopyFrom(packet);
    }

    _context->logFrame->set_blue_team(_context->game_state.blueTeam);
    _context->logFrame->set_command_time(RJ::timestamp());

    // TODO Debug drawing

    // TODO Radio RX/TX

    // Our robots
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& state = _context->world_state.our_robots.at(shell);
        const auto& status = _context->local_configs.at(shell);
        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = _context->logFrame->add_self();

        robot->mutable_pos()->set_x(state.pose.position().x());
        robot->mutable_pos()->set_y(state.pose.position().y());
        robot->set_angle(state.pose.heading());
        robot->mutable_world_vel()->set_x(state.velocity.linear().x());
        robot->mutable_world_vel()->set_y(state.velocity.linear().y());

        // TODO body vel?
        robot->set_ball_sense_status(BallSenseStatus::HasBall);
    }

    _context->logFrame->set_manual_id(_context->game_settings.joystick_config.manualID);
    _context->logFrame->set_defend_plus_x(_context->game_settings.defendPlusX);
    _context->logFrame->set_use_our_half(_context->game_settings.use_our_half);
    _context->logFrame->set_use_opponent_half(_context->game_settings.use_their_half);

    // TODO behavior tree

    // Team names
    if (_context->game_state.blueTeam) {
        _context->logFrame->set_team_name_yellow(_context->game_state.TheirInfo.name);
        _context->logFrame->set_team_name_blue(_context->game_state.OurInfo.name);
    } else {
        _context->logFrame->set_team_name_yellow(_context->game_state.OurInfo.name);
        _context->logFrame->set_team_name_blue(_context->game_state.TheirInfo.name);
    }

    _context->logFrame->set_timestamp(RJ::timestamp());
}

void Logger::stop() {
}

void Logger::read(const std::string& filename) {
    _log_file = std::fstream(filename);
    _state = State::kReading;
}

void Logger::write(const std::string& filename) {
    _log_file = std::fstream(filename);
    _state = State::kWriting;
}

void Logger::close() {
    _log_file = std::nullopt;
    _state = State::kNoFile;
}

void Logger::createLogFrame() {
    _context->logFrame = std::make_shared<LogFrame>();
}