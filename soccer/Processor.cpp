#include <gameplay/GameplayModule.hpp>

#include <poll.h>
#include <QMutexLocker>

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/messages_robocup_ssl_detection.pb.h>
#include <protobuf/messages_robocup_ssl_geometry.pb.h>
#include <protobuf/messages_robocup_ssl_wrapper.pb.h>
#include <Constants.hpp>
#include <Geometry2d/Util.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <Utils.hpp>
#include <joystick/GamepadController.hpp>
#include <joystick/GamepadJoystick.hpp>
#include <joystick/Joystick.hpp>
#include <joystick/SpaceNavJoystick.hpp>
#include <motion/MotionControl.hpp>
#include <multicast.hpp>
#include <planning/IndependentMultiRobotPathPlanner.hpp>
#include <rc-fshare/git_version.hpp>
#include "DebugDrawer.hpp"
#include "Processor.hpp"
#include "radio/NetworkRadio.hpp"
#include "radio/SimRadio.hpp"
#include "vision/VisionFilter.hpp"

REGISTER_CONFIGURABLE(Processor)

using namespace std;
using namespace boost;
using namespace Geometry2d;
using namespace google::protobuf;

static const auto Command_Latency = 0ms;

RobotConfig* Processor::robotConfig2008;
RobotConfig* Processor::robotConfig2011;
RobotConfig* Processor::robotConfig2015;
std::vector<RobotStatus*>
    Processor::robotStatuses;  ///< FIXME: verify that this is correct

Field_Dimensions* currentDimensions = &Field_Dimensions::Current_Dimensions;

void Processor::createConfiguration(Configuration* cfg) {
    robotConfig2008 = new RobotConfig(cfg, "Rev2008");
    robotConfig2011 = new RobotConfig(cfg, "Rev2011");
    robotConfig2015 = new RobotConfig(cfg, "Rev2015");

    for (size_t s = 0; s < Num_Shells; ++s) {
        robotStatuses.push_back(
            new RobotStatus(cfg, QString("Robot Statuses/Robot %1").arg(s)));
    }
}

Processor::Processor(bool sim, bool defendPlus, VisionChannel visionChannel,
                     bool blueTeam, std::string readLogFile="")
    : _loopMutex(), _blueTeam(blueTeam), _readLogFile(readLogFile) {
    _running = true;
    _manualID = -1;
    _framerate = 0;
    _useOurHalf = true;
    _useOpponentHalf = true;
    _initialized = false;
    _simulation = sim;
    _radio = nullptr;

    _multipleManual = false;
    setupJoysticks();

    _dampedTranslation = true;
    _dampedRotation = true;

    _kickOnBreakBeam = false;

    // Initialize team-space transformation
    defendPlusX(defendPlus);

    QMetaObject::connectSlotsByName(this);

    _vision = std::make_shared<VisionFilter>();
    _refereeModule = std::make_shared<NewRefereeModule>(&_context);
    _refereeModule->start();
    _gameplayModule = std::make_shared<Gameplay::GameplayModule>(&_context);
    _pathPlanner = std::unique_ptr<Planning::MultiRobotPathPlanner>(
        new Planning::IndependentMultiRobotPathPlanner());

    vision.simulation = _simulation;
    if (sim) {
        vision.port = SimVisionPort;
    }
    vision.start();

    _visionChannel = visionChannel;

    // Create radio socket
    _radio =
        _simulation
            ? static_cast<Radio*>(new SimRadio(&_context, _blueTeam))
            : static_cast<Radio*>(new NetworkRadio(NetworkRadioServerPort));

    if (!readLogFile.empty()) {
        _logger.readFrames(readLogFile.c_str());
        firstLogTime = _logger.startTime();
    }
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

void Processor::multipleManual(bool value) { _multipleManual = value; }

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

void Processor::joystickKickOnBreakBeam(bool value) {
    QMutexLocker locker(&_loopMutex);
    _kickOnBreakBeam = value;
}

void Processor::setupJoysticks() {
    _joysticks.clear();

    GamepadController::controllersInUse.clear();
    GamepadController::joystickRemoved = -1;

    for (int i = 0; i < Robots_Per_Team; i++) {
        _joysticks.push_back(new GamepadController());
    }

    //_joysticks.push_back(new SpaceNavJoystick()); //Add this back when
    // isValid() is working properly
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
        if (!externalReferee()) _refereeModule->blueTeam(value);
    }
}

bool Processor::joystickValid() const {
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) return true;
    }
    return false;
}

void Processor::runModels(const vector<const SSL_DetectionFrame*>& detectionFrames) {
    std::vector<CameraFrame> frames;

    for (const SSL_DetectionFrame* frame : detectionFrames) {
        vector<CameraBall> ballObservations;
        vector<CameraRobot> yellowObservations;
        vector<CameraRobot> blueObservations;

        RJ::Time time = RJ::Time(chrono::duration_cast<chrono::microseconds>(
            RJ::Seconds(frame->t_capture())));

        // Add ball observations
        ballObservations.reserve(frame->balls().size());
        for (const SSL_DetectionBall& ball : frame->balls()) {
            ballObservations.emplace_back(time, _worldToTeam * Point(ball.x() / 1000, ball.y() / 1000));
        }

        // Collect camera data from all robots
        yellowObservations.reserve(frame->robots_yellow().size());
        for (const SSL_DetectionRobot& robot : frame->robots_yellow()) {
            yellowObservations.emplace_back(time,
                                            _worldToTeam * Point(robot.x() / 1000, robot.y() / 1000),
                                            fixAngleRadians(robot.orientation() + _teamAngle),
                                            robot.robot_id());
        }

        // Collect camera data from all robots
        blueObservations.reserve(frame->robots_blue().size());
        for (const SSL_DetectionRobot& robot : frame->robots_blue()) {
            blueObservations.emplace_back(time,
                                          _worldToTeam * Point(robot.x() / 1000, robot.y() / 1000),
                                          fixAngleRadians(robot.orientation() + _teamAngle),
                                          robot.robot_id());
        }

        frames.emplace_back(time, frame->camera_id(), ballObservations, yellowObservations, blueObservations);
    }

    _vision->addFrames(frames);

    // Fill the list of our robots/balls based on whether we are the blue team or not
    _vision->fillBallState(_context.state);
    _vision->fillRobotState(_context.state, _blueTeam);
}

/**
 * program loop
 */
void Processor::run() {
    Status curStatus;

    bool first = true;
    // main loop
    while (_running) {
        RJ::Time startTime = RJ::now();
        auto deltaTime = startTime - curStatus.lastLoopTime;
        _framerate = RJ::Seconds(1) / deltaTime;
        curStatus.lastLoopTime = startTime;
        _context.state.time = startTime;

        if (!firstLogTime) {
            firstLogTime = startTime;
        }

        ////////////////
        // Reset

        // Make a new log frame
        _context.state.logFrame = std::make_shared<Packet::LogFrame>();
        _context.state.logFrame->set_timestamp(RJ::timestamp());
        _context.state.logFrame->set_command_time(
            RJ::timestamp(startTime + Command_Latency));
        _context.state.logFrame->set_use_our_half(_useOurHalf);
        _context.state.logFrame->set_use_opponent_half(_useOpponentHalf);
        _context.state.logFrame->set_manual_id(_manualID);
        _context.state.logFrame->set_blue_team(_blueTeam);
        _context.state.logFrame->set_defend_plus_x(_defendPlusX);
        _context.debug_drawer.setLogFrame(_context.state.logFrame.get());

        if (first) {
            first = false;

            Packet::LogConfig* logConfig =
                _context.state.logFrame->mutable_log_config();
            logConfig->set_generator("soccer");
            logConfig->set_git_version_hash(git_version_hash);
            logConfig->set_git_version_dirty(git_version_dirty);
            logConfig->set_simulation(_simulation);
        }

        for (OurRobot* robot : _context.state.self) {
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
                    break;
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
            SSL_WrapperPacket* log = _context.state.logFrame->add_raw_vision();
            log->CopyFrom(packet->wrapper);

            curStatus.lastVisionTime = packet->receivedTime;

            // If packet has geometry data, attempt to read information and
            // update if changed.
            if (packet->wrapper.has_geometry()) {
                updateGeometryPacket(packet->wrapper.geometry().field());
            }

            if (packet->wrapper.has_detection()) {
                SSL_DetectionFrame* det = packet->wrapper.mutable_detection();

                double rt =
                    RJ::numSeconds(packet->receivedTime.time_since_epoch());
                det->set_t_capture(rt - det->t_sent() + det->t_capture());
                det->set_t_sent(rt);

                // Remove balls on the excluded half of the field
                google::protobuf::RepeatedPtrField<SSL_DetectionBall>* balls =
                    det->mutable_balls();
                for (int i = 0; i < balls->size(); ++i) {
                    float x = balls->Get(i).x();
                    // FIXME - OMG too many terms
                    if ((!_context.state.logFrame->use_opponent_half() &&
                         ((_defendPlusX && x < 0) ||
                          (!_defendPlusX && x > 0))) ||
                        (!_context.state.logFrame->use_our_half() &&
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
                        if ((!_context.state.logFrame->use_opponent_half() &&
                             ((_defendPlusX && x < 0) ||
                              (!_defendPlusX && x > 0))) ||
                            (!_context.state.logFrame->use_our_half() &&
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

        while (_radio->hasReversePackets()) {
            Packet::RadioRx rx = _radio->popReversePacket();
            _context.state.logFrame->add_radio_rx()->CopyFrom(rx);

            curStatus.lastRadioRxTime =
                RJ::Time(chrono::microseconds(rx.timestamp()));

            // Store this packet in the appropriate robot
            unsigned int board = rx.robot_id();
            if (board < Num_Shells) {
                // We have to copy because the RX packet will survive past this
                // frame but LogFrame will not (the RadioRx in LogFrame will be
                // reused).
                _context.state.self[board]->setRadioRx(rx);
                _context.state.self[board]->radioRxUpdated();
            }
        }

        for (Joystick* joystick : _joysticks) {
            joystick->update();
        }
        GamepadController::joystickRemoved = -1;

        runModels(detectionFrames);
        for (VisionPacket* packet : visionPackets) {
            delete packet;
        }

        // Log referee data
        vector<NewRefereePacket*> refereePackets;
        _refereeModule.get()->getPackets(refereePackets);
        for (NewRefereePacket* packet : refereePackets) {
            SSL_Referee* log = _context.state.logFrame->add_raw_refbox();
            log->CopyFrom(packet->wrapper);
            curStatus.lastRefereeTime =
                std::max(curStatus.lastRefereeTime, packet->receivedTime);
            delete packet;
        }

        // Update gamestate w/ referee data
        _refereeModule->updateGameState(blueTeam());
        _refereeModule->spinKickWatcher();

        string yellowname, bluename;

        if (blueTeam()) {
            bluename = _context.game_state.OurInfo.name;
            yellowname = _context.game_state.TheirInfo.name;
        } else {
            yellowname = _context.game_state.OurInfo.name;
            bluename = _context.game_state.TheirInfo.name;
        }

        _context.state.logFrame->set_team_name_blue(bluename);
        _context.state.logFrame->set_team_name_yellow(yellowname);

        // Run high-level soccer logic
        _gameplayModule->run();

        // recalculates Field obstacles on every run through to account for
        // changing inset
        if (_gameplayModule->hasFieldEdgeInsetChanged()) {
            _gameplayModule->calculateFieldObstacles();
        }
        /// Collect global obstacles
        Geometry2d::ShapeSet globalObstacles =
            _gameplayModule->globalObstacles();
        Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
        Geometry2d::ShapeSet goalZoneObstacles =
            _gameplayModule->goalZoneObstacles();
        globalObstaclesWithGoalZones.add(goalZoneObstacles);

        // Build a plan request for each robot.
        std::map<int, Planning::PlanRequest> requests;
        for (OurRobot* r : _context.state.self) {
            if (r && r->visible) {
                if (_context.game_state.state == GameState::Halt) {
                    r->setPath(nullptr);
                    continue;
                }

                // Visualize local obstacles
                for (auto& shape : r->localObstacles().shapes()) {
                    _context.debug_drawer.drawShape(shape, Qt::black,
                                                    "LocalObstacles");
                }

                auto& globalObstaclesForBot =
                    (r->shell() == _gameplayModule->goalieID() ||
                     r->isPenaltyKicker || r->isBallPlacer)
                        ? globalObstacles
                        : globalObstaclesWithGoalZones;

                // create and visualize obstacles
                Geometry2d::ShapeSet staticObstacles =
                    r->collectStaticObstacles(
                        globalObstaclesForBot,
                        !(r->shell() == _gameplayModule->goalieID() ||
                          r->isPenaltyKicker || r->isBallPlacer));

                std::vector<Planning::DynamicObstacle> dynamicObstacles =
                    r->collectDynamicObstacles();

                requests.emplace(
                    r->shell(),
                    Planning::PlanRequest(
                        &_context, Planning::MotionInstant(r->pos, r->vel),
                        r->motionCommand()->clone(), r->robotConstraints(),
                        std::move(r->angleFunctionPath.path),
                        std::move(staticObstacles), std::move(dynamicObstacles),
                        r->shell(), r->getPlanningPriority()));
            }
        }

        // Run path planner and set the path for each robot that was planned for
        auto pathsById = _pathPlanner->run(std::move(requests));
        for (auto& entry : pathsById) {
            OurRobot* r = _context.state.self[entry.first];
            auto& path = entry.second;
            path->draw(&_context.debug_drawer, Qt::magenta, "Planning");
            path->drawDebugText(&_context.debug_drawer);
            r->setPath(std::move(path));

            r->angleFunctionPath.angleFunction =
                angleFunctionForCommandType(r->rotationCommand());
        }

        // Visualize obstacles
        for (auto& shape : globalObstacles.shapes()) {
            _context.debug_drawer.drawShape(shape, Qt::black,
                                            "Global Obstacles");
        }

        // Run velocity controllers
        for (OurRobot* robot : _context.state.self) {
            if (robot->visible) {
                if ((_manualID >= 0 && (int)robot->shell() == _manualID) ||
                    _context.game_state.halt()) {
                    robot->motionControl()->stopped();
                } else {
                    robot->motionControl()->run();
                }
            }
        }

        ////////////////
        // Store logging information

        // Debug layers
        const QStringList& layers = _context.debug_drawer.debugLayers();
        for (const QString& str : layers) {
            _context.state.logFrame->add_debug_layers(str.toStdString());
        }

        // Add our robots data to the LogFram
        for (OurRobot* r : _context.state.self) {
            if (r->visible) {
                r->addStatusText();

                Packet::LogFrame::Robot* log =
                    _context.state.logFrame->add_self();
                *log->mutable_pos() = r->pos;
                *log->mutable_world_vel() = r->vel;
                *log->mutable_body_vel() = r->vel.rotated(M_PI_2 - r->angle);
                //*log->mutable_cmd_body_vel() = r->
                // *log->mutable_cmd_vel() = r->cmd_vel;
                // log->set_cmd_w(r->cmd_w);
                log->set_shell(r->shell());
                log->set_angle(r->angle);
                auto radioRx = r->radioRx();
                if (radioRx.has_kicker_voltage()) {
                    log->set_kicker_voltage(radioRx.kicker_voltage());
                }

                if (radioRx.has_kicker_status()) {
                    log->set_charged(radioRx.kicker_status() & 0x01);
                    log->set_kicker_works(!(radioRx.kicker_status() & 0x90));
                }

                if (radioRx.has_ball_sense_status()) {
                    log->set_ball_sense_status(radioRx.ball_sense_status());
                }

                if (radioRx.has_battery()) {
                    log->set_battery_voltage(radioRx.battery());
                }

                log->mutable_motor_status()->Clear();
                log->mutable_motor_status()->MergeFrom(radioRx.motor_status());

                if (radioRx.has_quaternion()) {
                    log->mutable_quaternion()->Clear();
                    log->mutable_quaternion()->MergeFrom(radioRx.quaternion());
                } else {
                    log->clear_quaternion();
                }

                for (const Packet::DebugText& t : r->robotText) {
                    log->add_text()->CopyFrom(t);
                }
            }
        }

        // Opponent robots
        for (OpponentRobot* r : _context.state.opp) {
            if (r->visible) {
                Packet::LogFrame::Robot* log =
                    _context.state.logFrame->add_opp();
                *log->mutable_pos() = r->pos;
                log->set_shell(r->shell());
                log->set_angle(r->angle);
                *log->mutable_world_vel() = r->vel;
                *log->mutable_body_vel() = r->vel.rotated(2 * M_PI - r->angle);
            }
        }

        // Ball
        if (_context.state.ball.valid) {
            Packet::LogFrame::Ball* log =
                _context.state.logFrame->mutable_ball();
            *log->mutable_pos() = _context.state.ball.pos;
            *log->mutable_vel() = _context.state.ball.vel;
        }

        ////////////////
        // Outputs

        // Send motion commands to the robots
        sendRadioData();

        // Write to the log unless we are viewing logs
        if (_readLogFile.empty()) {
            _logger.addFrame(_context.state.logFrame);
        }

        // Store processing loop status
        _statusMutex.lock();
        _status = curStatus;
        _statusMutex.unlock();

        // Processor Initialization Completed
        _initialized = true;

        ////////////////
        // Timing

        auto endTime = RJ::now();
        auto timeLapse = endTime - startTime;
        if (timeLapse < _framePeriod) {
            // Use system usleep, not QThread::usleep.
            //
            // QThread::usleep uses pthread_cond_wait which sometimes fails to
            // unblock.
            // This seems to depend on how many threads are blocked.
            ::usleep(RJ::numMicroseconds(_framePeriod - timeLapse));
        } else {
            //   printf("Processor took too long: %d us\n", lastFrameTime);
        }
    }
    vision.stop();
}

/*
 * Updates the geometry packet if different from the existing one,
 * Based on the geometry vision data.
 */
void Processor::updateGeometryPacket(const SSL_GeometryFieldSize& fieldSize) {
    if (fieldSize.field_lines_size() == 0) {
        return;
    }

    const SSL_FieldCicularArc* penalty = nullptr;
    const SSL_FieldCicularArc* center = nullptr;
    float penaltyShortDist = 0;  // default value
    float penaltyLongDist = 0;   // default value
    float displacement =
        Field_Dimensions::Default_Dimensions.GoalFlat();  // default displacment

    // Loop through field arcs looking for needed fields
    for (const SSL_FieldCicularArc& arc : fieldSize.field_arcs()) {
        if (arc.name() == "CenterCircle") {
            // Assume center circle
            center = &arc;
        }
    }

    for (const SSL_FieldLineSegment& line : fieldSize.field_lines()) {
        if (line.name() == "RightPenaltyStretch") {
            displacement = abs(line.p2().y() - line.p1().y());
            penaltyLongDist = displacement;
        } else if (line.name() == "RightFieldRightPenaltyStretch") {
            penaltyShortDist = abs(line.p2().x() - line.p1().x());
        }
    }

    float thickness = fieldSize.field_lines().Get(0).thickness() / 1000.0f;

    // The values we get are the center of the lines, we want to use the
    // outside, so we can add this as an offset.
    float adj = fieldSize.field_lines().Get(0).thickness() / 1000.0f / 2.0f;

    float fieldBorder = currentDimensions->Border();

    if (penaltyLongDist != 0 && penaltyShortDist != 0 && center != nullptr &&
        thickness != 0) {
        // Force a resize
        Field_Dimensions newDim = Field_Dimensions(

            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f, fieldBorder, thickness,
            fieldSize.goal_width() / 1000.0f, fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            penaltyShortDist / 1000.0f,              // PenaltyShortDist
            penaltyLongDist / 1000.0f,               // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2));

        if (newDim != *currentDimensions) {
            setFieldDimensions(newDim);
        }
    } else if (center != nullptr && thickness != 0) {
        Field_Dimensions defaultDim = Field_Dimensions::Default_Dimensions;

        Field_Dimensions newDim = Field_Dimensions(
            fieldSize.field_length() / 1000.0f,
            fieldSize.field_width() / 1000.0f, fieldBorder, thickness,
            fieldSize.goal_width() / 1000.0f, fieldSize.goal_depth() / 1000.0f,
            Field_Dimensions::Default_Dimensions.GoalHeight(),
            defaultDim.PenaltyShortDist(),           // PenaltyShortDist
            defaultDim.PenaltyLongDist(),            // PenaltyLongDist
            center->radius() / 1000.0f + adj,        // CenterRadius
            (center->radius()) * 2 / 1000.0f + adj,  // CenterDiameter
            displacement / 1000.0f,                  // GoalFlat
            (fieldSize.field_length() / 1000.0f + (fieldBorder)*2),
            (fieldSize.field_width() / 1000.0f + (fieldBorder)*2));

        if (newDim != *currentDimensions) {
            setFieldDimensions(newDim);
        }
    } else {
        cerr << "Error: failed to decode SSL geometry packet. Not resizing "
                "field."
             << endl;
    }
}

void Processor::sendRadioData() {
    Packet::RadioTx* tx = _context.state.logFrame->mutable_radio_tx();
    tx->set_txmode(Packet::RadioTx::UNICAST);

    // Halt overrides normal motion control, but not joystick
    if (_context.game_state.halt()) {
        // Force all motor speeds to zero
        for (OurRobot* r : _context.state.self) {
            Packet::Control* control = r->control;
            control->set_xvelocity(0);
            control->set_yvelocity(0);
            control->set_avelocity(0);
            control->set_dvelocity(0);
            control->set_kcstrength(0);
            control->set_shootmode(Packet::Control::KICK);
            control->set_triggermode(Packet::Control::STAND_DOWN);
            control->set_song(Packet::Control::STOP);
        }
    }

    // Add RadioTx commands for visible robots and apply joystick input
    std::vector<int> manualIds = getJoystickRobotIds();
    for (OurRobot* r : _context.state.self) {
        if (r->visible || _manualID == r->shell() || _multipleManual) {
            Packet::Robot* txRobot = tx->add_robots();

            // Copy motor commands.
            // Even if we are using the joystick, this sets robot_id and the
            // number of motors.
            txRobot->CopyFrom(r->robotPacket);

            // MANUAL STUFF
            if (_multipleManual) {
                auto info =
                    find(manualIds.begin(), manualIds.end(), r->shell());
                int index = info - manualIds.begin();

                // figure out if this shell value has been assigned to a
                // joystick
                // do stuff with that information such as assign it to the first
                // available
                if (info == manualIds.end()) {
                    for (int i = 0; i < manualIds.size(); i++) {
                        if (manualIds[i] == -1) {
                            index = i;
                            _joysticks[i]->setRobotId(r->shell());
                            manualIds[i] = r->shell();
                            break;
                        }
                    }
                }

                if (index < manualIds.size()) {
                    applyJoystickControls(
                        getJoystickControlValue(*_joysticks[index]),
                        txRobot->mutable_control(), r);
                }
            } else if (_manualID == r->shell()) {
                auto controlValues = getJoystickControlValues();
                if (controlValues.size()) {
                    applyJoystickControls(controlValues[0],
                                          txRobot->mutable_control(), r);
                }
            }
        }
    }

    if (_radio) {
        _radio->send(*_context.state.logFrame->mutable_radio_tx());
    }
}

void Processor::applyJoystickControls(const JoystickControlValues& controlVals,
                                      Packet::Control* tx, OurRobot* robot) {
    Geometry2d::Point translation(controlVals.translation);

    // use world coordinates if we can see the robot
    // otherwise default to body coordinates
    if (robot && robot->visible && _useFieldOrientedManualDrive) {
        translation.rotate(-M_PI / 2 - robot->angle);
    }

    // translation
    tx->set_xvelocity(translation.x());
    tx->set_yvelocity(translation.y());

    // rotation
    tx->set_avelocity(controlVals.rotation);

    // kick/chip
    bool kick = controlVals.kick || controlVals.chip;
    tx->set_triggermode(kick
                            ? (_kickOnBreakBeam ? Packet::Control::ON_BREAK_BEAM
                                                : Packet::Control::IMMEDIATE)
                            : Packet::Control::STAND_DOWN);
    tx->set_kcstrength(controlVals.kickPower);
    tx->set_shootmode(controlVals.kick ? Packet::Control::KICK
                                       : Packet::Control::CHIP);

    // dribbler
    tx->set_dvelocity(controlVals.dribble ? controlVals.dribblerPower : 0);
}

JoystickControlValues Processor::getJoystickControlValue(Joystick& joy) {
    JoystickControlValues vals = joy.getJoystickControlValues();
    if (joy.valid()) {
        // keep it in range
        vals.translation.clamp(sqrt(2.0));
        if (vals.rotation > 1) vals.rotation = 1;
        if (vals.rotation < -1) vals.rotation = -1;

        // Gets values from the configured joystick control
        // values,respecting damped
        // state
        if (_dampedTranslation) {
            vals.translation *=
                Joystick::JoystickTranslationMaxDampedSpeed->value();
        } else {
            vals.translation *= Joystick::JoystickTranslationMaxSpeed->value();
        }
        if (_dampedRotation) {
            vals.rotation *= Joystick::JoystickRotationMaxDampedSpeed->value();
        } else {
            vals.rotation *= Joystick::JoystickRotationMaxSpeed->value();
        }

        // scale up kicker and dribbler speeds
        vals.dribblerPower *= Max_Dribble;
        vals.kickPower *= Max_Kick;
    }
    return vals;
}

std::vector<JoystickControlValues> Processor::getJoystickControlValues() {
    std::vector<JoystickControlValues> vals;
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            vals.push_back(getJoystickControlValue(*joy));
        }
    }
    return vals;
}

vector<int> Processor::getJoystickRobotIds() {
    vector<int> robotIds;
    for (Joystick* joy : _joysticks) {
        if (joy->valid()) {
            robotIds.push_back(joy->getRobotId());
        } else {
            robotIds.push_back(-2);
        }
    }
    return robotIds;
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
    cout << "Updating field geometry based off of vision packet." << endl;
    Field_Dimensions::Current_Dimensions = dims;
    recalculateWorldToTeamTransform();
    _gameplayModule->calculateFieldObstacles();
    _gameplayModule->updateFieldDimensions();
}

bool Processor::isRadioOpen() const { return _radio->isOpen(); }
bool Processor::isInitialized() const { return _initialized; }
