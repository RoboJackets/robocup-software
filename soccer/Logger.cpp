#include "Logger.hpp"

#include <fcntl.h>
#include <protobuf/referee.pb.h>
#include <stdio.h>
#include <unistd.h>

#include <QString>
#include <rc-fshare/git_version.hpp>
#include <time.hpp>

#include "Robot.hpp"
#include "Utils.hpp"

using namespace std;
using namespace Packet;
using namespace google::protobuf::io;

Logger::Logger(Context* context, size_t logSize)
    : _history(logSize), _context(context) {
    _fd = -1;
    _spaceUsed = sizeof(shared_ptr<Packet::LogFrame>) * _history.size();
}

Logger::~Logger() { close(); }

bool Logger::open(QString filename) {
    QWriteLocker locker(&_lock);

    if (_fd >= 0) {
        close();
    }

    _fd = creat(filename.toLatin1(), 0666);
    if (_fd < 0) {
        printf("Can't create %s: %m\n", (const char*)filename.toLatin1());
        return false;
    }

    _filename = filename;

    return true;
}

void Logger::close() {
    QWriteLocker locker(&_lock);
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
        _filename = QString();
    }
}

void Logger::addFrame(shared_ptr<LogFrame> frame) {
    this->addFrame(frame, false);
}

void Logger::addFrame(shared_ptr<LogFrame> frame, bool force) {
    QWriteLocker locker(&_lock);

    // Write this from to the file
    if (_fd >= 0) {
        if (frame->IsInitialized()) {
            uint32_t size = frame->ByteSizeLong();
            if (write(_fd, &size, sizeof(size)) != sizeof(size)) {
                printf("Logger: Failed to write size, closing log: %m\n");
                close();
            } else if (!frame->SerializeToFileDescriptor(_fd)) {
                printf("Logger: Failed to write frame, closing log: %m\n");
                close();
            }
        } else {
            printf("Logger: Not writing frame missing fields: %s\n",
                   frame->InitializationErrorString().c_str());
        }
    }

    if (_history.full() && !force) {
        _spaceUsed -= _history.front()->SpaceUsedLong();
        _history.pop_front();
    }
    // Create a new LogFrame

    _history.push_back(frame);

    // Add space used by the new data
    _spaceUsed += frame->SpaceUsedLong();
    _nextFrameNumber++;
}

shared_ptr<LogFrame> Logger::lastFrame() const {
    QReadLocker locker(&_lock);
    return _history.back();
}

void Logger::clear() {
    QReadLocker locker(&_lock);
    _history.clear();
    _spaceUsed = 0;
}

// Clears out existing logs
bool Logger::readFrames(const char* filename) {
    this->clear();

    _viewing = true;

    QFile file(filename);
    if (!file.open(QFile::ReadOnly)) {
        fprintf(stderr, "Can't open %s: %s\n", filename,
                (const char*)file.errorString().toLatin1());
        return false;
    }

    int n = 0;
    while (!file.atEnd()) {
        uint32_t size = 0;
        if (file.read((char*)&size, sizeof(size)) != sizeof(size)) {
            // Broken length
            printf("Broken length\n");
            return false;
        }

        string str(size, 0);
        if (file.read(&str[0], size) != size) {
            // Broken packet at end of file
            printf("Broken packet\n");
            return false;
        }

        std::shared_ptr<LogFrame> frame = std::make_shared<LogFrame>();
        if (!frame->ParsePartialFromString(str)) {
            printf("Failed: %s\n", frame->InitializationErrorString().c_str());
            return false;
        }
        this->addFrame(frame, true);
        ++n;
    }

    return true;
}

void Logger::start() {
    _startTime = RJ::now();

    // Make a new log frame for the next timestep.
    _context->logFrame = std::make_shared<Packet::LogFrame>();
    _context->logFrame->set_timestamp(RJ::timestamp());
    _context->logFrame->set_command_time(RJ::timestamp(RJ::now()));
    _context->logFrame->set_use_our_half(_context->game_settings.use_our_half);
    _context->logFrame->set_use_opponent_half(
        _context->game_settings.use_their_half);
    _context->logFrame->set_manual_id(_context->game_settings.manualID);
    _context->logFrame->set_blue_team(_context->game_state.blueTeam);
    _context->logFrame->set_defend_plus_x(_context->game_settings.defendPlusX);

    Packet::LogConfig* logConfig = _context->logFrame->mutable_log_config();
    logConfig->set_generator("soccer");
    logConfig->set_git_version_hash(git_version_hash);
    logConfig->set_git_version_dirty(git_version_dirty);
    logConfig->set_simulation(_context->game_settings.simulation);

    _context->debug_drawer.setLogFrame(_context->logFrame.get());
}

void Logger::run() {
    // We don't want to do any of this unless we're actually running
    // (not just viewing old logs).
    if (!_viewing) {
        // Consume referee packets
        std::vector<SSL_Referee> ref_packets =
            std::move(_context->referee_packets);
        _context->referee_packets.clear();

        for (const SSL_Referee& packet : ref_packets) {
            SSL_Referee* log = _context->logFrame->add_raw_refbox();
            log->CopyFrom(packet);
        }

        // Team names
        std::string yellowname, bluename;

        if (_context->game_state.blueTeam) {
            bluename = _context->game_state.OurInfo.name;
            yellowname = _context->game_state.TheirInfo.name;
        } else {
            yellowname = _context->game_state.OurInfo.name;
            bluename = _context->game_state.TheirInfo.name;
        }

        _context->logFrame->set_team_name_blue(bluename);
        _context->logFrame->set_team_name_yellow(yellowname);

        // Debug layers
        const QStringList& layers = _context->debug_drawer.debugLayers();
        for (const QString& str : layers) {
            _context->logFrame->add_debug_layers(str.toStdString());
        }

        // Add our robots data to the LogFrame
        for (OurRobot* r : _context->state.self) {
            if (r->visible()) {
                r->addStatusText();

                Packet::LogFrame::Robot* log_robot =
                    _context->logFrame->add_self();
                *log_robot->mutable_pos() = r->pos();
                *log_robot->mutable_world_vel() = r->vel();
                *log_robot->mutable_body_vel() =
                    r->vel().rotated(M_PI_2 - r->angle());
                log_robot->set_shell(r->shell());
                log_robot->set_angle(r->angle());
                auto radioRx = r->radioRx();
                if (radioRx.has_kicker_voltage()) {
                    log_robot->set_kicker_voltage(radioRx.kicker_voltage());
                }

                if (radioRx.has_kicker_status()) {
                    log_robot->set_charged(radioRx.kicker_status() & 0x01);
                    log_robot->set_kicker_works(
                        !(radioRx.kicker_status() & 0x90));
                }

                if (radioRx.has_ball_sense_status()) {
                    log_robot->set_ball_sense_status(
                        radioRx.ball_sense_status());
                }

                if (radioRx.has_battery()) {
                    log_robot->set_battery_voltage(radioRx.battery());
                }

                log_robot->mutable_motor_status()->Clear();
                log_robot->mutable_motor_status()->MergeFrom(
                    radioRx.motor_status());

                if (radioRx.has_quaternion()) {
                    log_robot->mutable_quaternion()->Clear();
                    log_robot->mutable_quaternion()->MergeFrom(
                        radioRx.quaternion());
                } else {
                    log_robot->clear_quaternion();
                }

                for (const Packet::DebugText& t : r->robotText) {
                    log_robot->add_text()->CopyFrom(t);
                }
            }
        }

        // Opponent robots
        for (OpponentRobot* r : _context->state.opp) {
            if (r->visible()) {
                Packet::LogFrame::Robot* log_robot =
                    _context->logFrame->add_opp();
                *log_robot->mutable_pos() = r->pos();
                log_robot->set_shell(r->shell());
                log_robot->set_angle(r->angle());
                *log_robot->mutable_world_vel() = r->vel();
                *log_robot->mutable_body_vel() =
                    r->vel().rotated(2 * M_PI - r->angle());
            }
        }

        // Ball
        if (_context->state.ball.valid) {
            Packet::LogFrame::Ball* log_ball =
                _context->logFrame->mutable_ball();
            *log_ball->mutable_pos() = _context->state.ball.pos;
            *log_ball->mutable_vel() = _context->state.ball.vel;
        }

        // Write to the log unless main window is paused
        if (!_context->game_settings.paused) {
            addFrame(_context->logFrame);
        }

        // Make a new log frame for the next timestep.
        _context->logFrame = std::make_shared<Packet::LogFrame>();
        _context->logFrame->set_timestamp(RJ::timestamp());
        _context->logFrame->set_command_time(RJ::timestamp(RJ::now()));
        _context->logFrame->set_use_our_half(
            _context->game_settings.use_our_half);
        _context->logFrame->set_use_opponent_half(
            _context->game_settings.use_their_half);
        _context->logFrame->set_manual_id(_context->game_settings.manualID);
        _context->logFrame->set_blue_team(_context->game_state.blueTeam);
        _context->logFrame->set_defend_plus_x(
            _context->game_settings.defendPlusX);

        _context->debug_drawer.setLogFrame(_context->logFrame.get());
    }
}
