#include "Logger.hpp"

#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "Context.hpp"
#include "radio/PacketConvert.hpp"

using namespace Packet;

// Protobuf helper function from: https://stackoverflow.com/a/22927149:
// From Comment:
// (I am the author of the C++ and Java protobuf libraries, but I no longer work
// for Google. Sorry that this code never made it into the official lib. This is
// what it would look like if it had.)

/**
 * Write a message to an output stream, delimited by size.
 */
bool writeDelimitedTo(const google::protobuf::MessageLite& message,
                      google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
    // We create a new coded stream for each message.  Don't worry, this is
    // fast.
    google::protobuf::io::CodedOutputStream output(rawOutput);

    // Write the size.
    const int size = message.ByteSize();
    output.WriteVarint32(size);

    uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
    if (buffer != nullptr) {
        // Optimization:  The message fits in one buffer, so use the faster
        // direct-to-array serialization path.
        message.SerializeWithCachedSizesToArray(buffer);
    } else {
        // Slightly-slower path when the message is multiple buffers.
        message.SerializeWithCachedSizes(&output);
        if (output.HadError()) {
            return false;
        }
    }

    return true;
}

/**
 * Read a message from an input stream, delimited by size.
 */
bool readDelimitedFrom(google::protobuf::io::ZeroCopyInputStream* rawInput,
                       google::protobuf::MessageLite* message) {
    // We create a new coded stream for each message.  Don't worry, this is
    // fast, and it makes sure the 64MB total size limit is imposed per-message
    // rather than on the whole stream.  (See the CodedInputStream interface for
    // more info on this limit.)
    google::protobuf::io::CodedInputStream input(rawInput);

    // Read the size.
    uint32_t size = 0;
    if (!input.ReadVarint32(&size)) {
        return false;
    }

    // Tell the stream not to read beyond that size.
    google::protobuf::io::CodedInputStream::Limit limit = input.PushLimit(size);

    // Parse the message.
    if (!message->MergeFromCodedStream(&input)) {
        return false;
    }
    if (!input.ConsumedEntireMessage()) {
        return false;
    }

    // Release the limit.
    input.PopLimit(limit);

    return true;
}

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

        robot->mutable_pos()->set_x(
            static_cast<float>(state.pose.position().x()));
        robot->mutable_pos()->set_y(
            static_cast<float>(state.pose.position().y()));
        robot->set_angle(static_cast<float>(state.pose.heading()));
        robot->mutable_world_vel()->set_x(
            static_cast<float>(state.velocity.linear().x()));
        robot->mutable_world_vel()->set_y(
            static_cast<float>(state.velocity.linear().y()));

        robot->set_shell(shell);

        robot->set_ball_sense_status(status.has_ball ? BallSenseStatus::HasBall
                                                     : BallSenseStatus::NoBall);

        robot->mutable_cmd_vel()->set_x(static_cast<float>(setpoint.xvelocity));
        robot->mutable_cmd_vel()->set_y(static_cast<float>(setpoint.yvelocity));
        robot->set_cmd_w(static_cast<float>(setpoint.avelocity));

        robot->set_charged(status.kicker == RobotStatus::KickerState::kCharged);
        robot->set_kicker_voltage(static_cast<float>(status.kicker_voltage));

        for (int i = 0; i < 5; i++) {
            robot->add_motor_status(status.motors_healthy[i]
                                        ? MotorStatus::Good
                                        : MotorStatus::Encoder_Failure);
        }

        robot->set_kicker_works(status.kicker !=
                                RobotStatus::KickerState::kFailed);
        robot->set_battery_voltage(static_cast<float>(status.battery_voltage));
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

    log_frame->mutable_radio_tx()->set_txmode(Packet::RadioTx_TxMode_MULTICAST);

    // Opponent robots
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& state = _context->world_state.their_robots.at(shell);
        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = log_frame->add_opp();

        robot->mutable_pos()->set_x(
            static_cast<float>(state.pose.position().x()));
        robot->mutable_pos()->set_y(
            static_cast<float>(state.pose.position().y()));
        robot->set_angle(static_cast<float>(state.pose.heading()));
        robot->mutable_world_vel()->set_x(
            static_cast<float>(state.velocity.linear().x()));
        robot->mutable_world_vel()->set_y(
            static_cast<float>(state.velocity.linear().y()));

        robot->set_shell(shell);
    }

    // Ball
    if (_context->state.ball.valid) {
        log_frame->mutable_ball()->mutable_pos()->set_x(
            static_cast<float>(_context->state.ball.pos.x()));
        log_frame->mutable_ball()->mutable_pos()->set_y(
            static_cast<float>(_context->state.ball.pos.y()));
        log_frame->mutable_ball()->mutable_vel()->set_x(
            static_cast<float>(_context->state.ball.vel.x()));
        log_frame->mutable_ball()->mutable_vel()->set_y(
            static_cast<float>(_context->state.ball.vel.y()));
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

    if (_context->logs.state == Logs::State::kWriting) {
        _context->logs.size_bytes += log_frame->ByteSizeLong();
        google::protobuf::io::OstreamOutputStream output(&_log_file.value());
        writeDelimitedTo(*log_frame, &output);
    }

    while (_context->logs.frames.size() + 1 >= kMaxLogFrames) {
        _context->logs.frames.pop_front();
    }

    _context->logs.frames.emplace_back(std::move(log_frame));
}

void Logger::stop() {}

void Logger::read(const std::string& filename) {
    _log_file = std::fstream();
    _log_file->open(filename, std::fstream::in | std::fstream::binary);
    _context->logs.filename = filename;
    _context->logs.state = Logs::State::kReading;

    _context->logs.frames.clear();

    if (!_log_file->good()) {
        std::cerr << "Log file " << filename << " does not exist." << std::endl;
        std::exit(-1);
    }

    // Populate the entire logs struct.
    google::protobuf::io::IstreamInputStream input(&_log_file.value());

    auto frame = std::make_shared<LogFrame>();
    while (readDelimitedFrom(&input, frame.get())) {
        _context->logs.size_bytes += frame->ByteSizeLong();
        _context->logs.frames.emplace_back(std::move(frame));
        frame = std::make_shared<LogFrame>();
    }
}

void Logger::write(const std::string& filename) {
    _log_file = std::fstream();
    _log_file->open(filename, std::fstream::out | std::fstream::binary);
    _context->logs.filename = filename;
    _context->logs.state = Logs::State::kWriting;

    for (const auto& frame : _context->logs.frames) {
        google::protobuf::io::OstreamOutputStream output(&_log_file.value());
        writeDelimitedTo(*frame, &output);
    }
}

void Logger::close() {
    _log_file = std::nullopt;
    _context->logs.filename = std::nullopt;
    _context->logs.state = Logs::State::kNoFile;
}
