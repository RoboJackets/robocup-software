#include "logger.hpp"

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <spdlog/spdlog.h>

#include <context.hpp>
#include <radio/packet_convert.hpp>
#include <rj_utils/logging.hpp>

#include <rc-fshare/git_version.hpp>

using namespace Packet;

// Protobuf helper function from: https://stackoverflow.com/a/22927149:
// From Comment:
// (I am the author of the C++ and Java protobuf libraries, but I no longer work
// for Google. Sorry that this code never made it into the official lib. This is
// what it would look like if it had.)

/**
 * Write a message to an output stream, delimited by size.
 */
bool write_delimited_to(const google::protobuf::MessageLite& message,
                        google::protobuf::io::ZeroCopyOutputStream* raw_output) {
    // We create a new coded stream for each message.  Don't worry, this is
    // fast.
    google::protobuf::io::CodedOutputStream output(raw_output);

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
bool read_delimited_from(google::protobuf::io::ZeroCopyInputStream* raw_input,
                         google::protobuf::MessageLite* message) {
    // We create a new coded stream for each message.  Don't worry, this is
    // fast, and it makes sure the 64MB total size limit is imposed per-message
    // rather than on the whole stream.  (See the CodedInputStream interface for
    // more info on this limit.)
    google::protobuf::io::CodedInputStream input(raw_input);

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

void Logger::start() {
    context_->logs.start_time = RJ::now();

    // Log a message that is empty except for a log config and a start time.
    std::shared_ptr<Packet::LogFrame> log_frame = std::make_shared<Packet::LogFrame>();
    log_frame->set_timestamp(
        std::chrono::duration_cast<std::chrono::microseconds>(RJ::now().time_since_epoch())
            .count());

    log_frame->mutable_log_config()->set_generator("soccer");
    log_frame->mutable_log_config()->set_git_version_hash(git_version_hash);
    log_frame->mutable_log_config()->set_git_version_dirty(git_version_dirty);
    log_frame->mutable_log_config()->set_simulation(context_->game_settings.simulation);
}

void Logger::run() {
    std::shared_ptr<Packet::LogFrame> log_frame = create_log_frame(context_);

    if (context_->logs.state == Logs::State::kWriting) {
        context_->logs.size_bytes += log_frame->ByteSize();
        google::protobuf::io::OstreamOutputStream output(&log_file_.value());
        write_to_file(log_frame.get(), &output);
    }

    while (context_->logs.frames.size() + 1 >= kMaxLogFrames) {
        context_->logs.frames.pop_front();
    }

    context_->logs.frames.emplace_back(std::move(log_frame));
}

void Logger::stop() {}

void Logger::read(const std::string& filename) {
    log_file_ = std::fstream();
    log_file_->open(filename, std::fstream::in | std::fstream::binary);
    context_->logs.filename = filename;
    context_->logs.state = Logs::State::kReading;

    context_->logs.frames.clear();

    if (!log_file_->good()) {
        FATAL_THROW("Log file {} does not exist.", filename);
    }

    // Populate the entire logs struct.
    auto frame = std::make_shared<LogFrame>();
    google::protobuf::io::IstreamInputStream input(&log_file_.value());
    while (read_from_file(frame.get(), &input)) {
        context_->logs.size_bytes += frame->ByteSize();
        context_->logs.frames.emplace_back(std::move(frame));
        frame = std::make_shared<LogFrame>();
    }
}

void Logger::write(const std::string& filename) {
    log_file_ = std::fstream();
    log_file_->open(filename, std::fstream::out | std::fstream::binary);
    context_->logs.filename = filename;
    context_->logs.state = Logs::State::kWriting;

    google::protobuf::io::OstreamOutputStream output(&log_file_.value());
    for (const auto& frame : context_->logs.frames) {
        write_to_file(frame.get(), &output);
    }
}

void Logger::close() {
    log_file_ = std::nullopt;
    context_->logs.filename = std::nullopt;
    context_->logs.state = Logs::State::kNoFile;
}

std::shared_ptr<Packet::LogFrame> Logger::create_log_frame(Context* context) {
    // Add everything to the log frame.
    auto log_frame = std::make_shared<LogFrame>();

    // Debug drawing
    context->debug_drawer.fill_log_frame(log_frame.get());

    // Copy raw vision packets.
    for (const SSL_WrapperPacket& packet : context->raw_vision_packets) {
        log_frame->add_raw_vision()->CopyFrom(packet);
    }
    context->raw_vision_packets.clear();

    // Copy referee packets.
    for (const auto& packet : context->referee_packets) {
        Referee* referee = log_frame->add_raw_refbox();
        referee->CopyFrom(packet);
    }
    context->referee_packets.clear();

    log_frame->set_blue_team(context->blue_team);
    log_frame->set_command_time(RJ::timestamp());

    // Our robots
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context->world_state.our_robots.at(shell);
        const auto& status = context->robot_status.at(shell);
        const auto& setpoint = context->motion_setpoints.at(shell);

        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = log_frame->add_self();
        fill_robot(robot, shell, &state, &status, &setpoint);
    }

    // Radio RX/TX for each robot
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& status = context->robot_status.at(shell);
        if (RJ::now() - status.timestamp > RJ::Seconds(0.5)) {
            continue;
        }
        const auto& intent = context->robot_intents.at(shell);
        const auto& setpoint = context->motion_setpoints.at(shell);

        RadioRx* rx = log_frame->add_radio_rx();
        ConvertRx::status_to_proto(status, rx);

        Packet::Robot* tx = log_frame->mutable_radio_tx()->add_robots();
        ConvertTx::to_proto(intent, setpoint, shell, tx);
    }

    // Opponent robots
    for (size_t shell = 0; shell < kNumShells; shell++) {
        const auto& state = context->world_state.their_robots.at(shell);
        if (!state.visible) {
            continue;
        }

        LogFrame::Robot* robot = log_frame->add_opp();
        fill_robot(robot, shell, &state, nullptr, nullptr);
    }

    // Ball
    if (context->world_state.ball.visible) {
        log_frame->mutable_ball()->mutable_pos()->set_x(
            static_cast<float>(context->world_state.ball.position.x()));
        log_frame->mutable_ball()->mutable_pos()->set_y(
            static_cast<float>(context->world_state.ball.position.y()));
        log_frame->mutable_ball()->mutable_vel()->set_x(
            static_cast<float>(context->world_state.ball.velocity.x()));
        log_frame->mutable_ball()->mutable_vel()->set_y(
            static_cast<float>(context->world_state.ball.velocity.y()));
    }

    log_frame->set_manual_id(context->game_settings.joystick_config.manual_id);
    log_frame->set_defend_plus_x(context->game_settings.defend_plus_x);
    log_frame->set_use_our_half(context->game_settings.use_our_half);
    log_frame->set_use_opponent_half(context->game_settings.use_their_half);

    // Behavior tree
    log_frame->set_behavior_tree(context->behavior_tree);

    // Team names
    if (context->blue_team) {
        log_frame->set_team_name_yellow(context->their_info.name);
        log_frame->set_team_name_blue(context->our_info.name);
    } else {
        log_frame->set_team_name_yellow(context->our_info.name);
        log_frame->set_team_name_blue(context->their_info.name);
    }

    log_frame->set_timestamp(RJ::timestamp());

    return log_frame;
}

bool Logger::write_to_file(Packet::LogFrame* frame,
                           google::protobuf::io::ZeroCopyOutputStream* out) {
    return write_delimited_to(*frame, out);
}

bool Logger::read_from_file(Packet::LogFrame* frame,
                            google::protobuf::io::ZeroCopyInputStream* in) {
    return read_delimited_from(in, frame);
}

void Logger::fill_robot(Packet::LogFrame::Robot* out, int shell_id, RobotState const* state,
                        RobotStatus const* status, MotionSetpoint const* setpoint) {
    out->set_shell(shell_id);

    if (state != nullptr) {
        out->mutable_pos()->set_x(static_cast<float>(state->pose.position().x()));
        out->mutable_pos()->set_y(static_cast<float>(state->pose.position().y()));
        out->set_angle(static_cast<float>(state->pose.heading()));
        out->mutable_world_vel()->set_x(static_cast<float>(state->velocity.linear().x()));
        out->mutable_world_vel()->set_y(static_cast<float>(state->velocity.linear().y()));
    }

    if (status != nullptr) {
        out->set_ball_sense_status(status->has_ball ? BallSenseStatus::HasBall
                                                    : BallSenseStatus::NoBall);

        out->set_charged(status->kicker == RobotStatus::KickerState::kCharged);
        out->set_kicker_voltage(static_cast<float>(status->kicker_voltage));

        for (int i = 0; i < 5; i++) {
            out->add_motor_status(status->motors_healthy[i] ? MotorStatus::Good
                                                            : MotorStatus::Encoder_Failure);
        }

        out->set_kicker_works(status->kicker != RobotStatus::KickerState::kFailed);
        out->set_battery_voltage(static_cast<float>(status->battery_voltage));
    }

    if (setpoint != nullptr) {
        out->mutable_cmd_vel()->set_x(static_cast<float>(setpoint->xvelocity));
        out->mutable_cmd_vel()->set_y(static_cast<float>(setpoint->yvelocity));
        out->set_cmd_w(static_cast<float>(setpoint->avelocity));
    }
}
