
#include <QMutexLocker>
#include <poll.h>

#include <gameplay/GameplayModule.hpp>
#include "Processor.hpp"
#include "radio/SimRadio.hpp"
#include "radio/USBRadio.hpp"
#include "modeling/BallTracker.hpp"
#include <multicast.hpp>
#include <Constants.hpp>
#include <Utils.hpp>
#include <joystick/Joystick.hpp>
#include <joystick/GamepadJoystick.hpp>
#include <joystick/SpaceNavJoystick.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <motion/MotionControl.hpp>
#include <RobotConfig.hpp>
#include <planning/IndependentMultiRobotPathPlanner.hpp>
#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <git_version.hpp>

REGISTER_CONFIGURABLE(Processor)

using namespace std;
using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

static const uint64_t Command_Latency = 0;

RobotConfig* Processor::robotConfig2008;
RobotConfig* Processor::robotConfig2011;
RobotConfig* Processor::robotConfig2015;
std::vector<RobotStatus*>
    Processor::robotStatuses;  ///< FIXME: verify that this is correct

// Joystick speed limits (for damped and non-damped mode)

void Processor::createConfiguration(Configuration* cfg) {
    robotConfig2008 = new RobotConfig(cfg, "Rev2008");
    robotConfig2011 = new RobotConfig(cfg, "Rev2011");
    robotConfig2015 = new RobotConfig(cfg, "Rev2015");

    for (size_t s = 0; s < Num_Shells; ++s) {
        robotStatuses.push_back(
            new RobotStatus(cfg, QString("Robot Statuses/Robot %1").arg(s)));
    }
}

Processor::Processor(bool sim) : _loopMutex(QMutex::Recursive) {
    _running = true;
    _framePeriod = 1000000 / 60;
    _manualID = -1;
    _defendPlusX = false;
    _externalReferee = true;
    _framerate = 0;
    firstLogTime = 0;
    _useOurHalf = true;
    _useOpponentHalf = true;

    _simulation = sim;
    _radio = nullptr;

    // joysticks
    _joysticks.push_back(new GamepadJoystick());
    _joysticks.push_back(new SpaceNavJoystick());
    _dampedTranslation = true;
    _dampedRotation = true;

    // Initialize team-space transformation
    defendPlusX(_defendPlusX);

    QMetaObject::connectSlotsByName(this);

    _ballTracker = std::make_shared<BallTracker>();
    _refereeModule = std::make_shared<NewRefereeModule>(_state);
    _refereeModule->start();
    _gameplayModule = std::make_shared<Gameplay::GameplayModule>(&_state);
    _pathPlanner = std::unique_ptr<Planning::MultiRobotPathPlanner>(
        new Planning::IndependentMultiRobotPathPlanner());
    vision.simulation = _simulation;
}

Processor::~Processor() {
    stop();

    for (Joystick* joy : _joysticks) {
        delete joy;
    }

    // DEBUG - This is unnecessary, but lets us determine which one breaks.
    //_refereeModule.reset();
    _gameplayModule.reset();
}

void Processor::stop() {
    if (_running) {
        _running = false;
        wait();
    }
}

void Processor::manualID(int value) {
    QMutexLocker locker(&_loopMutex);
    _manualID = value;

    for (Joystick* joy : _joysticks) {
        joy->reset();
    }
}

void Processor::goalieID(int value) {
    QMutexLocker locker(&_loopMutex);
    _gameplayModule->goalieID(value);
}

int Processor::goalieID() {
    QMutexLocker locker(&_loopMutex);
    return _gameplayModule->goalieID();
}

void Processor::dampedRotation(bool value) {
    QMutexLocker locker(&_loopMutex);
    _dampedRotation = value;
}

void Processor::dampedTranslation(bool value) {
    QMutexLocker locker(&_loopMutex);
    _dampedTranslation = value;
}

/**
 * sets the team
 * @param value the value indicates whether or not the current team is blue or
 * yellow
 */
void Processor::blueTeam(bool value) {
    // This is called from the GUI thread
    QMutexLocker locker(&_loopMutex);

    if (_blueTeam != value) {
        _blueTeam = value;
        if (_radio) _radio->switchTeam(_blueTeam);
        //_refereeModule->blueTeam(value);
    }
}

bool Processor::joystickValid() {
    QMutexLocker lock(&_loopMutex);
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) return true;
    }
    return false;
}

void Processor::runModels(
    const vector<const SSL_DetectionFrame*>& detectionFrames) {
    vector<BallObservation> ballObservations;

    for (const SSL_DetectionFrame* frame : detectionFrames) {
        RJ::Time time = RJ::SecsToTimestamp(frame->t_capture());

        // Add ball observations
        ballObservations.reserve(ballObservations.size() +
                                 frame->balls().size());
        for (const SSL_DetectionBall& ball : frame->balls()) {
            ballObservations.push_back(BallObservation(
                _worldToTeam * Point(ball.x() / 1000, ball.y() / 1000), time));
        }

        // Add robot observations
        const RepeatedPtrField<SSL_DetectionRobot>& selfRobots =
            _blueTeam ? frame->robots_blue() : frame->robots_yellow();
        for (const SSL_DetectionRobot& robot : selfRobots) {
            float angleRad = fixAngleRadians(robot.orientation() + _teamAngle);
            RobotObservation obs(
                _worldToTeam * Point(robot.x() / 1000, robot.y() / 1000),
                angleRad, time, frame->frame_number());
            obs.source = frame->camera_id();
            unsigned int id = robot.robot_id();
            if (id < _state.self.size()) {
                _state.self[id]->filter()->update(&obs);
            }
        }

        const RepeatedPtrField<SSL_DetectionRobot>& oppRobots =
            _blueTeam ? frame->robots_yellow() : frame->robots_blue();
        for (const SSL_DetectionRobot& robot : oppRobots) {
            float angleRad = fixAngleRadians(robot.orientation() + _teamAngle);
            RobotObservation obs(
                _worldToTeam * Point(robot.x() / 1000, robot.y() / 1000),
                angleRad, time, frame->frame_number());
            obs.source = frame->camera_id();
            unsigned int id = robot.robot_id();
            if (id < _state.opp.size()) {
                _state.opp[id]->filter()->update(&obs);
            }
        }
    }

    _ballTracker->run(ballObservations, &_state);

    for (Robot* robot : _state.self) {
        robot->filter()->predict(_state.logFrame->command_time(), robot);
    }

    for (Robot* robot : _state.opp) {
        robot->filter()->predict(_state.logFrame->command_time(), robot);
    }
}

/**
 * program loop
 */
void Processor::run() {
    vision.start();

    // Create radio socket
    _radio =
        _simulation ? (Radio*)new SimRadio(_blueTeam) : (Radio*)new USBRadio();

    Status curStatus;

    bool first = true;
    // main loop
    while (_running) {
        RJ::Time startTime = RJ::timestamp();
        int delta_us = startTime - curStatus.lastLoopTime;
        _framerate = 1000000.0 / delta_us;
        curStatus.lastLoopTime = startTime;
        _state.timestamp = startTime;

        if (!firstLogTime) {
            firstLogTime = startTime;
        }

        ////////////////
        // Reset

        // Make a new log frame
        _state.logFrame = std::make_shared<Packet::LogFrame>();
        _state.logFrame->set_timestamp(RJ::timestamp());
        _state.logFrame->set_command_time(startTime + Command_Latency);
        _state.logFrame->set_use_our_half(_useOurHalf);
        _state.logFrame->set_use_opponent_half(_useOpponentHalf);
        _state.logFrame->set_manual_id(_manualID);
        _state.logFrame->set_blue_team(_blueTeam);
        _state.logFrame->set_defend_plus_x(_defendPlusX);

        if (first) {
            first = false;

            Packet::LogConfig* logConfig =
                _state.logFrame->mutable_log_config();
            logConfig->set_generator("soccer");
            logConfig->set_git_version_hash(git_version_hash);
            logConfig->set_git_version_dirty(git_version_dirty);
            logConfig->set_simulation(_simulation);
        }

        for (OurRobot* robot : _state.self) {
            // overall robot config
            switch (robot->hardwareVersion()) {
                case Packet::RJ2008:
                    robot->config = robotConfig2008;
                    break;
                case Packet::RJ2011:
                    robot->config = robotConfig2011;
                    break;
                case Packet::RJ2015:
                    robot->config = robotConfig2015;
                case Packet::Unknown:
                    robot->config =
                        robotConfig2011;  // FIXME: defaults to 2011 robots
                    break;
            }

            // per-robot configs
            robot->status = robotStatuses.at(robot->shell());
        }

        ////////////////
        // Inputs

        // Read vision packets
        vector<const SSL_DetectionFrame*> detectionFrames;
        vector<VisionPacket*> visionPackets;
        vision.getPackets(visionPackets);
        for (VisionPacket* packet : visionPackets) {
            SSL_WrapperPacket* log = _state.logFrame->add_raw_vision();
            log->CopyFrom(packet->wrapper);

            curStatus.lastVisionTime = packet->receivedTime;
            if (packet->wrapper.has_detection()) {
                SSL_DetectionFrame* det = packet->wrapper.mutable_detection();

                // FIXME - Account for network latency
                double rt = packet->receivedTime / 1000000.0;
                det->set_t_capture(rt - det->t_sent() + det->t_capture());
                det->set_t_sent(rt);

                // Remove balls on the excluded half of the field
                google::protobuf::RepeatedPtrField<SSL_DetectionBall>* balls =
                    det->mutable_balls();
                for (int i = 0; i < balls->size(); ++i) {
                    float x = balls->Get(i).x();
                    // FIXME - OMG too many terms
                    if ((!_state.logFrame->use_opponent_half() &&
                         ((_defendPlusX && x < 0) ||
                          (!_defendPlusX && x > 0))) ||
                        (!_state.logFrame->use_our_half() &&
                         ((_defendPlusX && x > 0) ||
                          (!_defendPlusX && x < 0)))) {
                        balls->SwapElements(i, balls->size() - 1);
                        balls->RemoveLast();
                        --i;
                    }
                }

                // Remove robots on the excluded half of the field
                google::protobuf::RepeatedPtrField<SSL_DetectionRobot>*
                    robots[2] = {det->mutable_robots_yellow(),
                                 det->mutable_robots_blue()};

                for (int team = 0; team < 2; ++team) {
                    for (int i = 0; i < robots[team]->size(); ++i) {
                        float x = robots[team]->Get(i).x();
                        if ((!_state.logFrame->use_opponent_half() &&
                             ((_defendPlusX && x < 0) ||
                              (!_defendPlusX && x > 0))) ||
                            (!_state.logFrame->use_our_half() &&
                             ((_defendPlusX && x > 0) ||
                              (!_defendPlusX && x < 0)))) {
                            robots[team]->SwapElements(
                                i, robots[team]->size() - 1);
                            robots[team]->RemoveLast();
                            --i;
                        }
                    }
                }

                detectionFrames.push_back(det);
            }
        }

        // Read radio reverse packets
        _radio->receive();
        for (const Packet::RadioRx& rx : _radio->reversePackets()) {
            _state.logFrame->add_radio_rx()->CopyFrom(rx);

            curStatus.lastRadioRxTime = rx.timestamp();

            // Store this packet in the appropriate robot
            unsigned int board = rx.robot_id();
            if (board < Num_Shells) {
                // We have to copy because the RX packet will survive past this
                // frame but LogFrame will not (the RadioRx in LogFrame will be
                // reused).
                _state.self[board]->radioRx().CopyFrom(rx);
                _state.self[board]->radioRxUpdated();
            }
        }
        _radio->clear();

        _loopMutex.lock();

        for (Joystick* joystick : _joysticks) {
            joystick->update();
        }

        runModels(detectionFrames);
        for (VisionPacket* packet : visionPackets) {
            delete packet;
        }

        // Update gamestate w/ referee data
        _refereeModule->updateGameState(blueTeam());
        _refereeModule->spinKickWatcher();

        string yellowname, bluename;

        if (blueTeam()) {
            bluename = _state.gameState.OurInfo.name;
            yellowname = _state.gameState.TheirInfo.name;
        } else {
            yellowname = _state.gameState.OurInfo.name;
            bluename = _state.gameState.TheirInfo.name;
        }

        _state.logFrame->set_team_name_blue(bluename);
        _state.logFrame->set_team_name_yellow(yellowname);

        // Run high-level soccer logic
        _gameplayModule->run();

        /// Collect global obstacles
        Geometry2d::ShapeSet globalObstacles =
            _gameplayModule->globalObstacles();
        Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
        Geometry2d::ShapeSet goalZoneObstacles =
            _gameplayModule->goalZoneObstacles();
        globalObstaclesWithGoalZones.add(goalZoneObstacles);

        // Build a plan request for each robot.
        std::map<int, Planning::PlanRequest> requests;
        for (OurRobot* r : _state.self) {
            if (r && r->visible) {
                if (_state.gameState.state == GameState::Halt) {
                    r->setPath(nullptr);
                    continue;
                }

                // Visualize local obstacles
                for (auto& shape : r->localObstacles().shapes()) {
                    _state.drawShape(shape, Qt::black, "LocalObstacles");
                }

                auto& globalObstaclesForBot =
                    (r->shell() == _gameplayModule->goalieID() ||
                     r->isPenaltyKicker)
                        ? globalObstacles
                        : globalObstaclesWithGoalZones;

                // create and visualize obstacles
                Geometry2d::ShapeSet fullObstacles =
                    r->collectAllObstacles(globalObstaclesForBot);

                requests[r->shell()] = Planning::PlanRequest(
                    Planning::MotionInstant(r->pos, r->vel),
                    r->motionCommand()->clone(), r->motionConstraints(),
                    std::move(r->path()),
                    std::make_shared<ShapeSet>(std::move(fullObstacles)));
            }
        }

        // Run path planner and set the path for each robot that was planned for
        auto pathsById = _pathPlanner->run(std::move(requests));
        for (auto& entry : pathsById) {
            OurRobot* r = _state.self[entry.first];
            auto& path = entry.second;
            path->draw(&_state, Qt::magenta, "Planning");
            r->setPath(std::move(path));
        }

        // Visualize obstacles
        for (auto& shape : globalObstacles.shapes()) {
            _state.drawShape(shape, Qt::black, "Global Obstacles");
        }

        // Run velocity controllers
        for (OurRobot* robot : _state.self) {
            if (robot->visible) {
                if ((_manualID >= 0 && (int)robot->shell() == _manualID) ||
                    _state.gameState.halt()) {
                    robot->motionControl()->stopped();
                } else {
                    robot->motionControl()->run();
                }
            }
        }

        ////////////////
        // Store logging information

        // Debug layers
        const QStringList& layers = _state.debugLayers();
        for (const QString& str : layers) {
            _state.logFrame->add_debug_layers(str.toStdString());
        }

        // Add our robots data to the LogFram
        for (OurRobot* r : _state.self) {
            if (r->visible) {
                r->addStatusText();

                Packet::LogFrame::Robot* log = _state.logFrame->add_self();
                *log->mutable_pos() = r->pos;
                *log->mutable_world_vel() = r->vel;
                *log->mutable_body_vel() = r->vel.rotated(2 * M_PI - r->angle);
                //*log->mutable_cmd_body_vel() = r->
                // *log->mutable_cmd_vel() = r->cmd_vel;
                // log->set_cmd_w(r->cmd_w);
                log->set_shell(r->shell());
                log->set_angle(r->angle);

                if (r->radioRx().has_kicker_voltage()) {
                    log->set_kicker_voltage(r->radioRx().kicker_voltage());
                }

                if (r->radioRx().has_kicker_status()) {
                    log->set_charged(r->radioRx().kicker_status() & 0x01);
                    log->set_kicker_works(
                        !(r->radioRx().kicker_status() & 0x90));
                }

                if (r->radioRx().has_ball_sense_status()) {
                    log->set_ball_sense_status(
                        r->radioRx().ball_sense_status());
                }

                if (r->radioRx().has_battery()) {
                    log->set_battery_voltage(r->radioRx().battery());
                }

                log->mutable_motor_status()->Clear();
                log->mutable_motor_status()->MergeFrom(
                    r->radioRx().motor_status());

                if (r->radioRx().has_quaternion()) {
                    log->mutable_quaternion()->Clear();
                    log->mutable_quaternion()->MergeFrom(
                        r->radioRx().quaternion());
                } else {
                    log->clear_quaternion();
                }

                for (const Packet::DebugText& t : r->robotText) {
                    log->add_text()->CopyFrom(t);
                }
            }
        }

        // Opponent robots
        for (OpponentRobot* r : _state.opp) {
            if (r->visible) {
                Packet::LogFrame::Robot* log = _state.logFrame->add_opp();
                *log->mutable_pos() = r->pos;
                log->set_shell(r->shell());
                log->set_angle(r->angle);
                *log->mutable_world_vel() = r->vel;
                *log->mutable_body_vel() = r->vel.rotated(2 * M_PI - r->angle);
            }
        }

        // Ball
        if (_state.ball.valid) {
            Packet::LogFrame::Ball* log = _state.logFrame->mutable_ball();
            *log->mutable_pos() = _state.ball.pos;
            *log->mutable_vel() = _state.ball.vel;
        }

        ////////////////
        // Outputs

        // Send motion commands to the robots
        sendRadioData();

        // Write to the log
        _logger.addFrame(_state.logFrame);

        _loopMutex.unlock();

        // Store processing loop status
        _statusMutex.lock();
        _status = curStatus;
        _statusMutex.unlock();

        ////////////////
        // Timing

        RJ::Time endTime = RJ::timestamp();
        int lastFrameTime = endTime - startTime;
        if (lastFrameTime < _framePeriod) {
            // Use system usleep, not QThread::usleep.
            //
            // QThread::usleep uses pthread_cond_wait which sometimes fails to
            // unblock.
            // This seems to depend on how many threads are blocked.
            ::usleep(_framePeriod - lastFrameTime);
        } else {
            //   printf("Processor took too long: %d us\n", lastFrameTime);
        }
    }

    vision.stop();
}

void Processor::sendRadioData() {
    Packet::RadioTx* tx = _state.logFrame->mutable_radio_tx();

    // Halt overrides normal motion control, but not joystick
    if (_state.gameState.halt()) {
        // Force all motor speeds to zero
        for (OurRobot* r : _state.self) {
            Packet::RadioTx::Robot& txRobot = r->radioTx;
            txRobot.set_body_x(0);
            txRobot.set_body_y(0);
            txRobot.set_body_w(0);
            txRobot.set_kick(0);
            txRobot.set_dribbler(0);
        }
    }

    // Add RadioTx commands for visible robots and apply joystick input
    for (OurRobot* r : _state.self) {
        if (r->visible || _manualID == r->shell()) {
            Packet::RadioTx::Robot* txRobot = tx->add_robots();

            // Copy motor commands.
            // Even if we are using the joystick, this sets robot_id and the
            // number of motors.
            txRobot->CopyFrom(r->radioTx);

            if (r->shell() == _manualID) {
                JoystickControlValues controlVals = getJoystickControlValues();
                applyJoystickControls(controlVals, txRobot, r);
            }
        }
    }

    if (_radio) {
        _radio->send(*_state.logFrame->mutable_radio_tx());
    }
}

void Processor::applyJoystickControls(const JoystickControlValues& controlVals,
                                      Packet::RadioTx::Robot* tx,
                                      OurRobot* robot) {
    Geometry2d::Point translation(controlVals.translation);

    // use world coordinates if we can see the robot
    // otherwise default to body coordinates
    if (robot && robot->visible && _useFieldOrientedManualDrive) {
        translation.rotate(-robot->angle);
    } else {
        // adjust for robot coordinate system (x axis points forward through
        // the mouth of the bot)
        translation.rotate(-M_PI / 2.0f);
    }

    // translation
    tx->set_body_x(translation.x);
    tx->set_body_y(translation.y);

    // rotation
    tx->set_body_w(controlVals.rotation);

    // kick/chip
    bool kick = controlVals.kick || controlVals.chip;
    tx->set_kick_immediate(kick);
    tx->set_kick(kick ? controlVals.kickPower : 0);
    tx->set_use_chipper(controlVals.chip);

    // dribbler
    tx->set_dribbler(controlVals.dribble ? controlVals.dribblerPower : 0);
}

JoystickControlValues Processor::getJoystickControlValues() {
    // if there's more than one joystick, we add their values
    JoystickControlValues vals;
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            JoystickControlValues newVals = joy->getJoystickControlValues();

            if (newVals.dribble) vals.dribble = true;
            if (newVals.kick) vals.kick = true;
            if (newVals.chip) vals.chip = true;

            vals.rotation += newVals.rotation;
            vals.translation += newVals.translation;

            vals.dribblerPower =
                max<double>(vals.dribblerPower, newVals.dribblerPower);
            vals.kickPower = max<double>(vals.kickPower, newVals.kickPower);
        }
    }

    // keep it in range
    vals.translation.clamp(sqrt(2.0));
    if (vals.rotation > 1) vals.rotation = 1;
    if (vals.rotation < -1) vals.rotation = -1;

    // scale up speeds, respecting the damping modes
    if (_dampedTranslation) {
        vals.translation *=
            Joystick::JoystickTranslationMaxDampedSpeed->value();
    } else {
        vals.translation *= Joystick::JoystickRotationMaxSpeed->value();
    }
    if (_dampedRotation) {
        vals.rotation *= Joystick::JoystickRotationMaxDampedSpeed->value();
    } else {
        vals.rotation *= Joystick::JoystickRotationMaxSpeed->value();
    }

    // scale up kicker and dribbler speeds
    vals.dribblerPower *= 128;
    vals.kickPower *= 255;

    return vals;
}

void Processor::defendPlusX(bool value) {
    _defendPlusX = value;

    if (_defendPlusX) {
        _teamAngle = -M_PI_2;
    } else {
        _teamAngle = M_PI_2;
    }

    recalculateWorldToTeamTransform();
}

void Processor::changeVisionChannel(int port) {
    _loopMutex.lock();

    vision.stop();

    vision.simulation = _simulation;
    vision.port = port;
    vision.start();

    _loopMutex.unlock();
}

void Processor::recalculateWorldToTeamTransform() {
    _worldToTeam = Geometry2d::TransformMatrix::translate(
        0, Field_Dimensions::Current_Dimensions.Length() / 2.0f);
    _worldToTeam *= Geometry2d::TransformMatrix::rotate(_teamAngle);
}

void Processor::setFieldDimensions(const Field_Dimensions& dims) {
    Field_Dimensions::Current_Dimensions = dims;
    recalculateWorldToTeamTransform();
    _gameplayModule->calculateFieldObstacles();
    _gameplayModule->sendFieldDimensionsToPython();
}
